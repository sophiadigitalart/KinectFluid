#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"
#include "cinder/Log.h"
#include "cinder/Json.h"
#include <list>
#include "cinder/osc/Osc.h"
// Settings
#include "SDASettings.h"
// Session
#include "SDASession.h"
// Log
#include "SDALog.h"
// Spout
#include "CiSpoutOut.h"
// Fluid
#include "cinderfx/Fluid2D.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace SophiaDigitalArt;

using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;

const uint16_t localPort = 7000;

class KinectFluidApp : public App {

public:
	KinectFluidApp();
	void mouseMove(MouseEvent event) override;
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;
	void mouseUp(MouseEvent event) override;
	void keyDown(KeyEvent event) override;
	void keyUp(KeyEvent event) override;
	void touchesBegan(ci::app::TouchEvent event);
	void touchesMoved(ci::app::TouchEvent event);
	void touchesEnded(ci::app::TouchEvent event);
	void fileDrop(FileDropEvent event) override;
	void update() override;
	void draw() override;
	void cleanup() override;
	void setUIVisibility(bool visible);
private:
	// Settings
	SDASettingsRef					mSDASettings;
	// Session
	SDASessionRef					mSDASession;
	// Log
	SDALogRef						mSDALog;
	// imgui
	float							color[4];
	float							backcolor[4];
	int								playheadPositions[12];
	int								speeds[12];

	float							f = 0.0f;
	char							buf[64];
	unsigned int					i, j;

	bool							mouseGlobal;

	string							mError;
	// fbo
	bool							mIsShutDown;
	Anim<float>						mRenderWindowTimer;
	void							positionRenderWindow();
	bool							mFadeInDelay;
	SpoutOut 						mSpoutOut;
	// OSC
	Receiver						mReceiver;
	std::map<uint64_t, protocol::endpoint> mConnections;
	// kinect
	/*string jointNames[26] = { "SpineBase", "SpineMid", "Neck", "Head",
		"ShldrL", "ElbowL", "WristL", "HandL",
		"ShldrR", "ElbowR", "WristR", "HandR",
		"HipL", "KneeL", "AnkleL", "FootL",
		"HipR", "KneeR", "AnkleR", "FootR",
		"SpineShldr", "HandTipL", "ThumbL", "HandTipR", "ThumbR", "Count" };*/
	map<int, vec4>					mJoints;
	// Fluid
	ci::vec2						mPrevPos;
	ci::Colorf						mColor;

	std::map<int, ci::Colorf>		mTouchColors;
	float							mVelScale;
	float							mDenScale;
	float							mRgbScale;
	cinderfx::Fluid2D				mFluid2D;
	ci::gl::Texture2dRef			mTex;

};


KinectFluidApp::KinectFluidApp()
	: mSpoutOut("SDAKinectFluid", app::getWindowSize()),
	mReceiver(localPort)
{
	// Settings
	mSDASettings = SDASettings::create();
	// Session
	mSDASession = SDASession::create(mSDASettings);
	//mSDASettings->mCursorVisible = true;
	setUIVisibility(mSDASettings->mCursorVisible);
	mSDASession->getWindowsResolution();

	mouseGlobal = false;
	mFadeInDelay = true;

	// OSC
	// ? is body 0 to 5 
	mReceiver.setListener("/?/*",
		[&](const osc::Message &message) {
		float x = message[0].flt();
		float y = message[1].flt();
		float z = message[2].flt();
		int jointIndex = message[3].int32() + 200;
		int bodyIndex = message[4].int32();
		string jointName = message[5].string();
		stringstream sParams;
		sParams << "{\"k2\" :[{\"name\":\"" << toString(jointIndex) << "\",\"value\":\"" << toString(x) << "," << toString(y) << "," << toString(z) << "," << toString(bodyIndex) << "\"}]}";
		mSDASession->wsWrite(sParams.str());
		if (jointIndex == 211) CI_LOG_W(sParams.str());
		mJoints[message[3].int32()] = vec4(x * getWindowWidth() + getWindowWidth() / 2,
			y * getWindowHeight() + getWindowHeight() / 2, z, bodyIndex);

	});
	try {
		// Bind the receiver to the endpoint. This function may throw.
		mReceiver.bind();
	}
	catch (const osc::Exception &ex) {
		CI_LOG_E("Error binding: " << ex.what() << " val: " << ex.value());
		quit();
	}
	// UDP opens the socket and "listens" accepting any message from any endpoint. The listen
	// function takes an error handler for the underlying socket. Any errors that would
	// call this function are because of problems with the socket or with the remote message.
	mReceiver.listen(
		[](asio::error_code error, protocol::endpoint endpoint) -> bool {
		if (error) {
			CI_LOG_E("Error Listening: " << error.message() << " val: " << error.value() << " endpoint: " << endpoint);
			return false;
		}
		else
			return true;
	});

	// Fluid
	glEnable(GL_TEXTURE_2D);

	mFluid2D.enableDensity();
	mFluid2D.enableRgb();
	mFluid2D.enableVorticityConfinement();

	mDenScale = 50;
	mRgbScale = 50;

	mFluid2D.set(192, 192);
	mFluid2D.setDensityDissipation(0.99f);
	mFluid2D.setRgbDissipation(0.99f);
	mVelScale = 3.0f*std::max(mFluid2D.resX(), mFluid2D.resY());

	// windows
	mIsShutDown = false;
	mRenderWindowTimer = 0.0f;
	//timeline().apply(&mRenderWindowTimer, 1.0f, 2.0f).finishFn([&] { positionRenderWindow(); });

}
void KinectFluidApp::positionRenderWindow() {
	mSDASettings->mRenderPosXY = ivec2(mSDASettings->mRenderX, mSDASettings->mRenderY);//20141214 was 0
	setWindowPos(mSDASettings->mRenderX, mSDASettings->mRenderY);
	setWindowSize(mSDASettings->mRenderWidth, mSDASettings->mRenderHeight);
}
void KinectFluidApp::setUIVisibility(bool visible)
{
	if (visible)
	{
		showCursor();
	}
	else
	{
		hideCursor();
	}
}
void KinectFluidApp::fileDrop(FileDropEvent event)
{
	mSDASession->fileDrop(event);
}
void KinectFluidApp::update()
{
	mSDASession->setFloatUniformValueByIndex(mSDASettings->IFPS, getAverageFps());
	mSDASession->update();
	mFluid2D.step();
}
void KinectFluidApp::cleanup()
{
	if (!mIsShutDown)
	{
		mIsShutDown = true;
		CI_LOG_V("shutdown");
		// save settings
		mSDASettings->save();
		mSDASession->save();
		quit();
	}
}
void KinectFluidApp::mouseMove(MouseEvent event)
{
	
}
void KinectFluidApp::mouseDown(MouseEvent event)
{
	mPrevPos = event.getPos();
	mColor.r = Rand::randFloat();
	mColor.g = Rand::randFloat();
	mColor.b = Rand::randFloat();
}
void KinectFluidApp::mouseDrag(MouseEvent event)
{
	float x = (event.getX() / (float)getWindowWidth())*mFluid2D.resX();
	float y = (event.getY() / (float)getWindowHeight())*mFluid2D.resY();

	if (event.isLeftDown()) {
		vec2 dv = vec2(event.getPos()) - mPrevPos;
		mFluid2D.splatVelocity(x, y, mVelScale*dv);
		mFluid2D.splatRgb(x, y, mRgbScale*mColor);
		if (mFluid2D.isBuoyancyEnabled()) {
			mFluid2D.splatDensity(x, y, mDenScale);
		}
	}
	mPrevPos = event.getPos();
}
void KinectFluidApp::mouseUp(MouseEvent event)
{
	
}

void KinectFluidApp::keyDown(KeyEvent event)
{
	if (!mSDASession->handleKeyDown(event)) {
		switch (event.getCode()) {
		case KeyEvent::KEY_ESCAPE:
			// quit the application
			quit();
			break;
		case KeyEvent::KEY_h:
			// mouse cursor and ui visibility
			mSDASettings->mCursorVisible = !mSDASettings->mCursorVisible;
			setUIVisibility(mSDASettings->mCursorVisible);
			break;
		}
	}
}
void KinectFluidApp::keyUp(KeyEvent event)
{
	
}
void KinectFluidApp::touchesBegan(TouchEvent event)
{
	const std::vector<TouchEvent::Touch>& touches = event.getTouches();
	for (std::vector<TouchEvent::Touch>::const_iterator cit = touches.begin(); cit != touches.end(); ++cit) {
		Colorf color;
		color.r = Rand::randFloat();
		color.g = Rand::randFloat();
		color.b = Rand::randFloat();
		mTouchColors[cit->getId()] = color;
	}
}

void KinectFluidApp::touchesMoved(TouchEvent event)
{
	const std::vector<TouchEvent::Touch>& touches = event.getTouches();
	for (std::vector<TouchEvent::Touch>::const_iterator cit = touches.begin(); cit != touches.end(); ++cit) {
		if (mTouchColors.find(cit->getId()) == mTouchColors.end())
			continue;
		vec2 prevPos = cit->getPrevPos();
		vec2 pos = cit->getPos();
		float x = (pos.x / (float)getWindowWidth())*mFluid2D.resX();
		float y = (pos.y / (float)getWindowHeight())*mFluid2D.resY();
		vec2 dv = pos - prevPos;
		mFluid2D.splatVelocity(x, y, mVelScale*dv);
		mFluid2D.splatRgb(x, y, mRgbScale*mTouchColors[cit->getId()]);
		if (mFluid2D.isBuoyancyEnabled()) {
			mFluid2D.splatDensity(x, y, mDenScale);
		}
	}
}

void KinectFluidApp::touchesEnded(TouchEvent event)
{
	const std::vector<TouchEvent::Touch>& touches = event.getTouches();
	for (std::vector<TouchEvent::Touch>::const_iterator cit = touches.begin(); cit != touches.end(); ++cit) {
		mTouchColors.erase(cit->getId());
	}
}

void KinectFluidApp::draw()
{
	gl::clear(Color::black());
	/*if (mFadeInDelay) {
		mSDASettings->iAlpha = 0.0f;
		if (getElapsedFrames() > mSDASession->getFadeInDelay()) {
			mFadeInDelay = false;
			timeline().apply(&mSDASettings->iAlpha, 0.0f, 1.0f, 1.5f, EaseInCubic());
		}
	}

	gl::setMatricesWindow(getWindowWidth(), getWindowHeight(), false);
	for (unsigned i = 0; i < mJoints.size(); ++i)
	{
		gl::drawSolidCircle(mJoints[i], 10);
	}	*/
	float* data = const_cast<float*>((float*)mFluid2D.rgb().data());
	Surface32f surf(data, mFluid2D.resX(), mFluid2D.resY(), mFluid2D.resX() * sizeof(Colorf), SurfaceChannelOrder::RGB);

	if (!mTex) {
		mTex = gl::Texture::create(surf);
	}
	else {
		mTex->update(surf);
	}
	gl::draw(mTex, getWindowBounds());
	// Spout Send
	mSpoutOut.sendViewport();

	getWindow()->setTitle(mSDASettings->sFps + " KinectFluid");
}

void prepareSettings(App::Settings *settings)
{
#if defined( CINDER_MSW )
	//settings->setConsoleWindowEnabled();
#endif
	settings->setMultiTouchEnabled();

	settings->setWindowSize(854, 480);
}

CINDER_APP(KinectFluidApp, RendererGl, prepareSettings)
