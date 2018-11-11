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
// UserInterface
#define IM_ARRAYSIZE(_ARR)			((int)(sizeof(_ARR)/sizeof(*_ARR)))
#define IMGUI_DISABLE_OBSOLETE_FUNCTIONS
#include "CinderImGui.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace SophiaDigitalArt;

using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;

const uint16_t localPort = 7000;
struct Joints
{
	string		name;	
	vec2		mPos;
	vec2		mPrevPos;
};
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
	void resize() override;
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
	bool							mIsResizing;

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
	map<int, Joints>					mJoints;
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
	mJoints[0].name = "SpineBase";
	mJoints[1].name = "SpineMid";
	mJoints[2].name = "Neck";
	mJoints[3].name = "Head";
	mJoints[4].name = "ShldrL";
	mJoints[5].name = "ElbowL";
	mJoints[6].name = "WristL";
	mJoints[7].name = "HandL";
	mJoints[8].name = "ShldrR";
	mJoints[9].name = "ElbowR";
	mJoints[10].name = "WristR";
	mJoints[11].name = "HandR";
	mJoints[12].name = "HipL";
	mJoints[13].name = "KneeL";
	mJoints[14].name = "AnkleL";
	mJoints[15].name = "FootL";
	mJoints[16].name = "HipR";
	mJoints[17].name = "KneeR";
	mJoints[18].name = "AnkleR";
	mJoints[19].name = "FootR";
	mJoints[20].name = "SpineShldr";
	mJoints[21].name = "HandTipL";
	mJoints[22].name = "ThumbL";
	mJoints[23].name = "HandTipR";
	mJoints[24].name = "ThumbR";
	mJoints[25].name = "Count";
	for (unsigned i = 0; i < mJoints.size(); i++)
	{
		mJoints[i].mPrevPos.x = 10;
		mJoints[i].mPrevPos.y = 10;
		mJoints[i].mPos.x = 200;
		mJoints[i].mPos.y = 200;
	}
 

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
		mJoints[message[3].int32()].mPos.x = (x + 1.0) * mFluid2D.resX();
		mJoints[message[3].int32()].mPos.y = (y + 1.0) * mFluid2D.resY();
		sParams << " jx " << toString(mJoints[message[3].int32()].mPos.x) << " jy " << toString(mJoints[message[3].int32()].mPos.y);

		//if (jointIndex == 211) 
			CI_LOG_W(sParams.str());
	
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
	mIsResizing = false;
}
void KinectFluidApp::resize() {
	mIsResizing = true;
	// disconnect ui window and io events callbacks
	ImGui::disconnectWindow(getWindow());
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
	for (unsigned i = 0; i < mJoints.size(); i++)
	{
		Colorf color;
		color.r = Rand::randFloat();
		color.g = Rand::randFloat();
		color.b = Rand::randFloat();
		float x = (mJoints[i].mPos.x / (float)getWindowWidth())*mFluid2D.resX();
		float y = (mJoints[i].mPos.y / (float)getWindowHeight())*mFluid2D.resY();
		vec2 dv = mJoints[i].mPos - mJoints[i].mPrevPos;
		mFluid2D.splatVelocity(x, y, mVelScale*dv);
		mFluid2D.splatRgb(x, y, mRgbScale*color);
		if (mFluid2D.isBuoyancyEnabled()) {
			mFluid2D.splatDensity(x, y, mDenScale);
		}
		mJoints[i].mPrevPos.x = mJoints[i].mPos.x;
		mJoints[i].mPrevPos.y = mJoints[i].mPos.y;
	}
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
	/*

	gl::setMatricesWindow(getWindowWidth(), getWindowHeight(), false);
	for (unsigned i = 0; i < mJoints.size(); ++i)
	{
		gl::drawSolidCircle(mJoints[i], 10);
	}	*/
	
	/**/
	float* data = const_cast<float*>((float*)mFluid2D.rgb().data());
	Surface32f surf(data, mFluid2D.resX(), mFluid2D.resY(), mFluid2D.resX() * sizeof(Colorf), SurfaceChannelOrder::RGB);

	if (!mTex) {
		mTex = gl::Texture::create(surf);
	}
	else {
		mTex->update(surf);
	}
	gl::draw(mTex, getWindowBounds());
	for (unsigned i = 0; i < mJoints.size(); i++)
	{
		gl::drawSolidCircle(vec2(mJoints[i].mPos.x, mJoints[i].mPos.y), 10);
	}
	// Spout Send
	mSpoutOut.sendViewport();

	ImGuiStyle& style = ImGui::GetStyle();

	//if (mIsResizing) {
	//	mIsResizing = false;

		// set ui window and io events callbacks 
		ImGui::connectWindow(getWindow());
		ImGui::initialize();

#pragma region style
		// our theme variables
		style.WindowRounding = 4;
		style.WindowPadding = ImVec2(3, 3);
		style.FramePadding = ImVec2(2, 2);
		style.ItemSpacing = ImVec2(3, 3);
		style.ItemInnerSpacing = ImVec2(3, 3);
		style.WindowMinSize = ImVec2(300, 300);
		style.Alpha = 0.85f;
		style.Colors[ImGuiCol_Text] = ImVec4(0.89f, 0.92f, 0.94f, 1.00f);
		style.Colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.05f, 0.05f, 1.00f);
		style.Colors[ImGuiCol_Border] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
		style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.38f);
		style.Colors[ImGuiCol_FrameBg] = ImVec4(0.18f, 0.18f, 0.18f, 1.00f);
		style.Colors[ImGuiCol_TitleBg] = ImVec4(0.4f, 0.0f, 0.21f, 1.00f);
		style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.17f, 0.17f, 0.17f, 1.00f);
		style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.97f, 0.0f, 0.17f, 1.00f);
		style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
		style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
		style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.26f, 0.26f, 0.26f, 1.00f);
		style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.26f, 0.26f, 0.26f, 1.00f);
		// style.Colors[ImGuiCol_ComboBg] = ImVec4(0.13f, 0.13f, 0.13f, 1.00f);
		style.Colors[ImGuiCol_CheckMark] = ImVec4(0.99f, 0.22f, 0.22f, 0.50f);
		style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.65f, 0.25f, 0.25f, 1.00f);
		style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.8f, 0.35f, 0.35f, 1.00f);
		style.Colors[ImGuiCol_Button] = ImVec4(0.17f, 0.17f, 0.17f, 1.00f);
		style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.27f, 0.27f, 0.27f, 1.00f);
		style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
		style.Colors[ImGuiCol_Header] = ImVec4(0.11f, 0.11f, 0.11f, 1.00f);
		style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
		style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.27f, 0.27f, 0.27f, 1.00f);
		style.Colors[ImGuiCol_Column] = ImVec4(0.04f, 0.04f, 0.04f, 0.22f);
		style.Colors[ImGuiCol_ColumnHovered] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
		style.Colors[ImGuiCol_ColumnActive] = ImVec4(0.27f, 0.27f, 0.27f, 1.00f);
		style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.65f, 0.25f, 0.25f, 1.00f);
		style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.8f, 0.35f, 0.35f, 1.00f);
		style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.9f, 0.45f, 0.45f, 1.00f);
		style.Colors[ImGuiCol_CloseButton] = ImVec4(0.28f, 0.28f, 0.28f, 1.00f);
		style.Colors[ImGuiCol_CloseButtonHovered] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
		style.Colors[ImGuiCol_CloseButtonActive] = ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
		style.Colors[ImGuiCol_PlotLines] = ImVec4(0.65f, 0.25f, 0.25f, 1.00f);
		style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.8f, 0.35f, 0.35f, 1.00f);
		style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.65f, 0.25f, 0.25f, 1.00f);
		style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.8f, 0.35f, 0.35f, 1.00f);
		style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.24f, 0.24f, 0.24f, 1.00f);
#pragma endregion style
		ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiSetCond_Once);
		ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiSetCond_Once);
		sprintf(buf, "KinectFluid Fps %c %d###fps", "|/-\\"[(int)(ImGui::GetTime() / 0.25f) & 3], (double)getAverageFps());
		ImGui::Begin(buf);
		{
			ImGui::SliderFloat("hlx", &mJoints[7].mPos.x, 0.00f, 300.0f);
			ImGui::SliderFloat("hly", &mJoints[7].mPos.y, 0.00f, 300.0f);
			ImGui::SliderFloat("hrx", &mJoints[11].mPos.x, 0.00f, 300.0f);
			ImGui::SliderFloat("hry", &mJoints[11].mPos.y, 0.00f, 300.0f);
		}
		ImGui::End();
	//}
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
