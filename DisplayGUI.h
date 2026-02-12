//---------------------------------------------------------------------------

#ifndef DisplayGUIH
#define DisplayGUIH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include "Components\OpenGLv0.5BDS2006\Component\OpenGLPanel.h"
#include <ComCtrls.hpp>
#include <ExtCtrls.hpp>
#include <Menus.hpp>
#include <IdBaseComponent.hpp>
#include <IdComponent.hpp>
#include <Graphics.hpp>
#include "FilesystemStorage.h"
#include "KeyholeConnection.h"
#include "GoogleLayer.h"
#include "FlatEarthView.h"
#include "ght_hash_table.h"
#include "TriangulatPoly.h"
#include "Aircraft.h"
#include <Dialogs.hpp>
#include <IdTCPClient.hpp>
#include <IdTCPConnection.hpp>
#include "cspin.h"
#include <System.Net.HttpClient.hpp>
#include <System.Net.HttpClientComponent.hpp>
#include <System.Net.URLClient.hpp>
#include <System.SyncObjs.hpp>
#include "SpeechLib_OCX.h"
#include <Vcl.OleServer.hpp>
#include <deque>
#include <map>
#include <vector>

typedef float T_GL_Color[4];


typedef struct
{
 bool Valid_CC;
 bool Valid_CPA;
 uint32_t ICAO_CC;
 uint32_t ICAO_CPA;
}TTrackHook;

typedef struct
{
 double lat;
 double lon;
 double hae;
}TPolyLine;

typedef struct
{
 double lat;
 double lon;
}TGeoPoint;

typedef enum
{
 RFS_NONE=0,
 RFS_PENDING,
 RFS_READY,
 RFS_FAILED
}TRouteFetchStatus;

typedef struct
{
 TRouteFetchStatus Status;
 AnsiString RouteText;
 AnsiString DestCode;
 bool HaveDestination;
 double DestLat;
 double DestLon;
 __int64 LastAttemptMs;
}TRouteCacheEntry;

typedef struct
{
 bool Valid;
 uint32_t ICAO;
 std::vector<TGeoPoint> Points;
 double LastLat;
 double LastLon;
 double LastHeading;
 double LastSpeed;
 AnsiString LastRouteKey;
 bool HadDestination;
}TTrajectoryState;

typedef struct
{
 bool Valid;
 uint32_t ICAO;
 AnsiString Signature;
 std::vector<TGeoPoint> Points;
 double Co2BestKg;
 double Co2GeodesicKg;
 double Co2ReductionKg;
}TCO2OptimalState;

typedef struct
{
 AnsiString Signature;
 uint32_t ICAO;
 double StartLat;
 double StartLon;
 double EndLat;
 double EndLon;
}TCO2RecommendRequest;


#define MAX_AREA_POINTS 500
typedef struct
{
 AnsiString  Name;
 TColor      Color;
 DWORD       NumPoints;
 pfVec3      Points[MAX_AREA_POINTS];
 pfVec3      PointsAdj[MAX_AREA_POINTS];
 TTriangles *Triangles;
 bool        Selected;
}TArea;
//---------------------------------------------------------------------------
class  TTCPClientRawHandleThread : public TThread
{
private:
	AnsiString StringMsgBuffer;
	void __fastcall HandleInput(void);
	void __fastcall StopPlayback(void);
	void __fastcall StopTCPClient(void);
protected:
	void __fastcall Execute(void);
public:
	 bool UseFileInsteadOfNetwork;
	 bool First;
	 __int64 LastTime;
	__fastcall TTCPClientRawHandleThread(bool value);
	~TTCPClientRawHandleThread();
};
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
class  TTCPClientSBSHandleThread : public TThread
{
private:
	AnsiString StringMsgBuffer;
	void __fastcall HandleInput(void);
	void __fastcall StopPlayback(void);
	void __fastcall StopTCPClient(void);
protected:
	void __fastcall Execute(void);
public:
	 bool UseFileInsteadOfNetwork;
	 bool First;
	 __int64 LastTime;
	__fastcall TTCPClientSBSHandleThread(bool value);
	~TTCPClientSBSHandleThread();
};
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
class TRouteFetchThread : public TThread
{
private:
	AnsiString FlightNum;
	void __fastcall DoFetch(void);
protected:
	void __fastcall Execute(void);
public:
	__fastcall TRouteFetchThread(bool value);
	~TRouteFetchThread();
};

// Background worker for CO2-optimal trajectory recommendation.
class TCO2RecommendThread : public TThread
{
private:
	TCO2RecommendRequest Req;
	void __fastcall DoFetch(void);
protected:
	void __fastcall Execute(void);
public:
	__fastcall TCO2RecommendThread(bool value);
	~TCO2RecommendThread();
};
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
class TForm1 : public TForm
{
__published:	// IDE-managed Components
	TMainMenu *MainMenu1;
	TPanel *RightPanel;
	TMenuItem *File1;
	TMenuItem *Exit1;
	TTimer *Timer1;
	TOpenGLPanel *ObjectDisplay;
	TPanel *Panel1;
	TPanel *Panel3;
	TButton *ZoomIn;
	TButton *ZoomOut;
	TCheckBox *DrawMap;
	TCheckBox *PurgeStale;
	TTimer *Timer2;
	TCSpinEdit *CSpinStaleTime;
	TButton *PurgeButton;
	TListView *AreaListView;
	TButton *Insert;
	TButton *Delete;
	TButton *Complete;
	TButton *Cancel;
	TButton *RawConnectButton;
	TLabel *Label16;
	TLabel *Label17;
	TEdit *RawIpAddress;
	TIdTCPClient *IdTCPClientRaw;
	TSaveDialog *RecordRawSaveDialog;
	TOpenDialog *PlaybackRawDialog;
	TCheckBox *CycleImages;
	TPanel *Panel4;
	TLabel *CLatLabel;
	TLabel *CLonLabel;
	TLabel *SpdLabel;
	TLabel *HdgLabel;
	TLabel *AltLabel;
	TLabel *MsgCntLabel;
	TLabel *TrkLastUpdateTimeLabel;
	TLabel *Label14;
	TLabel *Label13;
	TLabel *Label10;
	TLabel *Label9;
	TLabel *Label8;
	TLabel *Label7;
	TLabel *Label6;
	TLabel *Label18;
	TLabel *FlightNumLabel;
	TLabel *ICAOLabel;
	TLabel *Label5;
	TLabel *Label4;
	TPanel *Panel5;
	TLabel *Lon;
	TLabel *Label3;
	TLabel *Lat;
	TLabel *Label2;
	TStaticText *SystemTime;
	TLabel *SystemTimeLabel;
	TLabel *ViewableAircraftCountLabel;
	TLabel *AircraftCountLabel;
	TLabel *Label11;
	TLabel *Label1;
	TButton *RawPlaybackButton;
	TButton *RawRecordButton;
	TIdTCPClient *IdTCPClientSBS;
	TButton *SBSConnectButton;
	TEdit *SBSIpAddress;
	TButton *SBSRecordButton;
	TButton *SBSPlaybackButton;
	TSaveDialog *RecordSBSSaveDialog;
	TOpenDialog *PlaybackSBSDialog;
	TTrackBar *TimeToGoTrackBar;
	TCheckBox *TimeToGoCheckBox;
	TStaticText *TimeToGoText;
	TLabel *Label12;
	TLabel *Label19;
	TLabel *CpaTimeValue;
	TLabel *CpaDistanceValue;
	TPanel *Panel2;
	TComboBox *MapComboBox;
	TCheckBox *BigQueryCheckBox;
	TMenuItem *UseSBSLocal;
	TMenuItem *UseSBSRemote;
	TMenuItem *LoadARTCCBoundaries1;
	TNetHTTPClient *NetHTTPClientRoute;
	TLabel *Label20;
	TLabel *RouteLabel;
	TSpVoice *SpVoice1;
	TSpSharedRecoContext *SpSharedRecoContext1;
	TMemo *Memo1;
	TMenuItem *LIsten;
	void __fastcall ObjectDisplayInit(TObject *Sender);
	void __fastcall ObjectDisplayResize(TObject *Sender);
	void __fastcall ObjectDisplayPaint(TObject *Sender);
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall ResetXYOffset(void);
	void __fastcall ObjectDisplayMouseDown(TObject *Sender, TMouseButton Button,
		  TShiftState Shift, int X, int Y);
	void __fastcall ObjectDisplayMouseMove(TObject *Sender, TShiftState Shift,
		  int X, int Y);
	void __fastcall AddPoint(int X, int Y);	  
	void __fastcall ObjectDisplayMouseUp(TObject *Sender, TMouseButton Button,
          TShiftState Shift, int X, int Y);
	void __fastcall Exit1Click(TObject *Sender);
	void __fastcall ZoomInClick(TObject *Sender);
	void __fastcall ZoomOutClick(TObject *Sender);
	void __fastcall Timer2Timer(TObject *Sender);
	void __fastcall PurgeButtonClick(TObject *Sender);
	void __fastcall InsertClick(TObject *Sender);
	void __fastcall CancelClick(TObject *Sender);
	void __fastcall CompleteClick(TObject *Sender);
	void __fastcall AreaListViewSelectItem(TObject *Sender, TListItem *Item,
          bool Selected);
	void __fastcall DeleteClick(TObject *Sender);
	void __fastcall AreaListViewCustomDrawItem(TCustomListView *Sender,
          TListItem *Item, TCustomDrawState State, bool &DefaultDraw);
	void __fastcall FormMouseWheel(TObject *Sender, TShiftState Shift,
          int WheelDelta, TPoint &MousePos, bool &Handled);
	void __fastcall RawConnectButtonClick(TObject *Sender);
	void __fastcall IdTCPClientRawConnected(TObject *Sender);
	void __fastcall RawRecordButtonClick(TObject *Sender);
	void __fastcall RawPlaybackButtonClick(TObject *Sender);
	void __fastcall IdTCPClientRawDisconnected(TObject *Sender);
	void __fastcall CycleImagesClick(TObject *Sender);
	void __fastcall SBSConnectButtonClick(TObject *Sender);
	void __fastcall SBSRecordButtonClick(TObject *Sender);
	void __fastcall SBSPlaybackButtonClick(TObject *Sender);
	void __fastcall IdTCPClientSBSConnected(TObject *Sender);
	void __fastcall IdTCPClientSBSDisconnected(TObject *Sender);
	void __fastcall TimeToGoTrackBarChange(TObject *Sender);
	void __fastcall MapComboBoxChange(TObject *Sender);
	void __fastcall BigQueryCheckBoxClick(TObject *Sender);
	void __fastcall UseSBSRemoteClick(TObject *Sender);
	void __fastcall UseSBSLocalClick(TObject *Sender);
	void __fastcall LoadARTCCBoundaries1Click(TObject *Sender);
	void __fastcall SpSharedRecoContext1Recognition(TObject *Sender, long StreamNumber,
          Variant StreamPosition, SpeechRecognitionType RecognitionType,
          ISpeechRecoResult *Result);
	void __fastcall LIstenClick(TObject *Sender);
	void __fastcall SpeechPopupToggleButtonClick(TObject *Sender);
	void __fastcall SpeechPopupTimerTick(TObject *Sender);
	void __fastcall SpeechPopupWavePaint(TObject *Sender);
	void __fastcall SpeechPopupClose(TObject *Sender, TCloseAction &Action);
    void __fastcall NetHTTPClientPredictionRequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse);
	void __fastcall NetHTTPClientPredictionRequestError(TObject *Sender, const UnicodeString AError);
	void __fastcall NetHTTPClientWeatherRequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse);
    void __fastcall NetHTTPClientWeatherRequestError(TObject *Sender, const UnicodeString AError);
    void __fastcall NetHTTPClientFuelRequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse);
	void __fastcall NetHTTPClientFuelRequestError(TObject *Sender, const UnicodeString AError);
	void __fastcall NetHTTPClientSpeechQARequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse);
	void __fastcall NetHTTPClientSpeechQARequestError(TObject *Sender, const UnicodeString AError);

	private:	// User declarations
	friend class TRouteFetchThread;
	friend class TCO2RecommendThread;
	void __fastcall OpenSpeechPopup();
	void __fastcall CreateSpeechPopup();
	void __fastcall UpdateSpeechPopupUi(const AnsiString &statusText);
	void __fastcall BuildWhisperPaths();
	void __fastcall StartWhisperFromPopup();
	void __fastcall StopWhisperFromPopup();
	void __fastcall PollWhisperProcessFromPopup();
	void __fastcall FinalizeWhisperRun(DWORD exitCode, bool forcedStop);
	void __fastcall CleanupWhisperHandles();
	void __fastcall CloseSpeechPopupAndStopProcess(bool forceTerminate);
	AnsiString __fastcall FormatSpeechElapsed(DWORD elapsedMs) const;
	void __fastcall AppendPopupMemoLine(const AnsiString &line);
	void __fastcall AppendWhisperLogTail(int maxLines);
	AnsiString __fastcall NormalizeCodeToken(const AnsiString &value);
	AnsiString __fastcall NormalizeFlightNum(const AnsiString &value);
	bool __fastcall DequeueRouteFetch(AnsiString &flightNum);
	bool __fastcall DequeueCO2Recommend(TCO2RecommendRequest &req);
	void __fastcall StoreRouteFetchResult(const AnsiString &flightNum,
										  bool success,
										  const AnsiString &routeText,
										  const AnsiString &destCode,
										  bool haveDestination,
										  double destLat,
										  double destLon);
	void __fastcall StoreCO2RecommendResult(const TCO2RecommendRequest &req,
									   bool success,
									   const std::vector<TGeoPoint> &points,
									   double co2BestKg,
									   double co2GeodesicKg,
									   double co2ReductionKg);
	void __fastcall RequestFuelSummary(const AnsiString &icaoHex,
									  const AnsiString &flightId,
									  const AnsiString &phase,
									  bool hasCurrentState = false,
									  double currentLat = 0.0,
									  double currentLon = 0.0,
									  double currentGsKt = 0.0,
									  const AnsiString &routeText = "");
	void __fastcall RequestSpeechQa(const AnsiString &transcript);

	public:		// User declarations
	__fastcall TForm1(TComponent* Owner);
	__fastcall ~TForm1();
	void __fastcall LatLon2XY(double lat,double lon, double &x, double &y);
	int __fastcall  XY2LatLon2(int x, int y,double &lat,double &lon );
	void __fastcall HookTrack(int X, int Y,bool CPA_Hook);
	void __fastcall DrawObjects(void);
	void __fastcall DeleteAllAreas(void);
	void __fastcall Purge(void);
	void __fastcall SendCotMessage(AnsiString IP_address, unsigned short Port,char *Buffer,DWORD Length);
	void __fastcall RegisterWithCoTRouter(void);
    void __fastcall SetMapCenter(double &x, double &y);
    void __fastcall LoadMap(int Type);
	void __fastcall CreateBigQueryCSV(void);
	void __fastcall CloseBigQueryCSV(void);
	bool __fastcall LoadARTCCBoundaries(AnsiString FileName);
	void __fastcall EnqueueRouteFetch(const AnsiString &flightNum);
	void __fastcall EnqueueCO2Recommend(const TCO2RecommendRequest &req);
	bool __fastcall TryGetCachedRoute(const AnsiString &flightNum, TRouteCacheEntry &entry);
	bool __fastcall ParseDestinationCode(const AnsiString &routeText, AnsiString &destCode);
	bool __fastcall BuildTrajectory(const TADS_B_Aircraft *data, const TRouteCacheEntry *routeEntry,
									 std::vector<TGeoPoint> &outPoints, bool &hasDestination);
	void __fastcall DrawTrajectory(const std::vector<TGeoPoint> &points, bool hasDestination);
	void __fastcall DrawOptimalTrajectory(const std::vector<TGeoPoint> &points);
	void __fastcall ClearTrajectoryState();
	AnsiString __fastcall BuildPredictedRoutePointsJsonFromPoints(const std::vector<TGeoPoint> &points);
	void __fastcall SendPredictedRoutePointsToBackendFromPoints(const std::vector<TGeoPoint> &points);
	AnsiString __fastcall BuildPredictedRoutePointsJson(TADS_B_Aircraft *Data);
	void __fastcall SendPredictedRoutePointsToBackend(TADS_B_Aircraft *Data);

    TLabel                     *PhaseLabel;
	TLabel                     *Co2Label;
	TLabel                     *FuelRateLabel;
	TLabel                     *FuelUsedLabel;
	TLabel                     *FuelCo2Label;
    TNetHTTPClient             *NetHTTPClientPrediction;
    TNetHTTPClient             *NetHTTPClientWeather;
	TNetHTTPClient             *NetHTTPClientFuel;
	TNetHTTPClient             *NetHTTPClientSpeechQA;
    ISpeechRecoGrammar         *SRGrammar;
	int                        MouseDownX,MouseDownY;
	bool                       MouseDown;
	TTrackHook                 TrackHook;
	Vector3d                   Map_v[4],Map_p[4];
	Vector2d                   Map_w[2];
	double                     Mw1,Mw2,Mh1,Mh2,xf,yf;
	KeyholeConnection	      *g_Keyhole;
	FilesystemStorage	      *g_Storage;
	MasterLayer	      	      *g_MasterLayer;
	TileManager		          *g_GETileManager;
	EarthView		          *g_EarthView;
	double                     MapCenterLat,MapCenterLon;
	int			               g_MouseLeftDownX;
	int			               g_MouseLeftDownY;
	int			               g_MouseDownMask ;
	bool                       LoadMapFromInternet;
	TList                     *Areas;
	TArea                     *AreaTemp;
	ght_hash_table_t          *HashTable;
	TTCPClientRawHandleThread *TCPClientRawHandleThread;
    TTCPClientSBSHandleThread *TCPClientSBSHandleThread;
	TStreamWriter              *RecordRawStream;
	TStreamReader              *PlayBackRawStream;
    TStreamWriter              *RecordSBSStream;
	TStreamReader              *PlayBackSBSStream;
	TStreamWriter              *BigQueryCSV;
    AnsiString                 BigQueryCSVFileName;
	unsigned int               BigQueryRowCount;
	unsigned int               BigQueryFileCount;
    AnsiString                 BigQueryPythonScript;
	AnsiString                 BigQueryPath;
    AnsiString                 BigQueryLogFileName;
	int                        NumSpriteImages;
	int                        CurrentSpriteImage;
    AnsiString                 AircraftDBPathFileName;
    AnsiString                 ARTCCBoundaryDataPathFileName;
	AnsiString                 AirportLookupPathFileName;
	std::map<AnsiString, TRouteCacheEntry> RouteCache;
	std::deque<AnsiString>     RouteFetchQueue;
	std::deque<TCO2RecommendRequest> CO2RecommendQueue;
	System::Syncobjs::TCriticalSection *RouteCacheCs;
	System::Syncobjs::TCriticalSection *CO2Cs;
	TRouteFetchThread         *RouteFetchThread;
	TCO2RecommendThread       *CO2RecommendThread;
	TTrajectoryState           TrajectoryState;
	TCO2OptimalState           CO2OptimalState;
	__int64                    LastWeatherOverlayPostMs;
	AnsiString                 LastWeatherOverlaySignature;
	__int64                    LastCO2RecommendPostMs;
	AnsiString                 LastCO2RecommendSignature;
	__int64                    LastFuelSummaryPostMs;
	AnsiString                 LastFuelSummarySignature;
	AnsiString                 LastFuelSummaryPhase;
	AnsiString                 LastFuelSummaryRoute;
	TForm                     *SpeechPopup;
	TButton                   *SpeechToggleButton;
	TLabel                    *SpeechPopupStatusLabel;
	TLabel                    *SpeechPopupTimerLabel;
	TPaintBox                 *SpeechWavePaintBox;
	TMemo                     *SpeechPopupMemo;
	TTimer                    *SpeechPopupTimer;
	bool                       WhisperRecording;
	bool                       WhisperProcessing;
	PROCESS_INFORMATION        WhisperPi;
	HANDLE                     WhisperLogHandle;
	DWORD                      WhisperRecordStartTick;
	int                        SpeechWaveTick;
	AnsiString                 WhisperToolsDir;
	AnsiString                 WhisperScriptFile;
	AnsiString                 WhisperStopFile;
	AnsiString                 WhisperOutFile;
	AnsiString                 WhisperLogFile;
	AnsiString                 WhisperAudioFile;
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------


#endif
