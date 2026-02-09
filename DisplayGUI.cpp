//---------------------------------------------------------------------------

#include <vcl.h>
#include <new>
#include <math.h>
#include <dir.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <ctype.h>
#include <filesystem>
#include <fileapi.h>
#include <memory>
#include <string>
#include <vector>
#include <System.JSON.hpp>

#pragma hdrstop

#include "DisplayGUI.h"
#include "AreaDialog.h"
#include "ntds2d.h"
#include "LatLonConv.h"
#include "PointInPolygon.h"
#include "DecodeRawADS_B.h"
#include "ght_hash_table.h"
#include "dms.h"
#include "Aircraft.h"
#include "TimeFunctions.h"
#include "SBS_Message.h"
#include "CPA.h"
#include "AircraftDB.h"
#include "AirportLookup.h"
#include "csv.h"

#define AIRCRAFT_DATABASE_URL   "https://opensky-network.org/datasets/metadata/aircraftDatabase.zip"
#define AIRCRAFT_DATABASE_FILE   "aircraftDatabase.csv"
#define ARTCC_BOUNDARY_FILE      "Ground_Level_ARTCC_Boundary_Data_2025-05-15.csv"
//"https://vrs-standing-data.adsb.lol/routes.csv.gz"
#define API_SERVICE_URL_JSON  "https://vrs-standing-data.adsb.lol/routes/%.2s/%s.json"
#define API_SERVICE_URL_TXT  "https://vrs-standing-data.adsb.lol/routes/%.2s/%s.txt"
#define WEATHER_OVERLAY_URL "http://localhost:8001/weather/overlay"
#define MAP_CENTER_LAT  40.73612;
#define MAP_CENTER_LON -80.33158;

#define BIG_QUERY_UPLOAD_COUNT 50000
#define BIG_QUERY_RUN_FILENAME  "SimpleCSVtoBigQuery.py"
#define   LEFT_MOUSE_DOWN   1
#define   RIGHT_MOUSE_DOWN  2
#define   MIDDLE_MOUSE_DOWN 4
#define TRAJECTORY_LEAD_SECONDS 60.0
#define TRAJECTORY_POINT_SPACING_SECONDS 30.0
#define TRAJECTORY_MAX_POINTS 200
#define ROUTE_FETCH_RETRY_COOLDOWN_MS 30000
#define TURN_DT_SECONDS 1.0
#define TURN_RATE_DEG_PER_SEC 3.0
#define HEADING_ALIGN_DEG 2.0
#define TURN_PHASE_MAX_SECONDS 300
#define ARRIVAL_THRESHOLD_NM 0.5
#define WEATHER_POST_THROTTLE_MS 2000


#define BG_INTENSITY   0.37
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "OpenGLPanel"
#pragma link "Map\libgefetch\Win64\Release\libgefetch.a"
#pragma link "Map\zlib\Win64\Release\zlib.a"
#pragma link "Map\jpeg\Win64\Release\jpeg.a"
#pragma link "Map\png\Win64\Release\png.a"
#pragma link "HashTable\Lib\Win64\Release\HashTableLib.a"
#pragma link "cspin"
#pragma link "SpeechLib_OCX"
#pragma resource "*.dfm"
TForm1 *Form1;
 //---------------------------------------------------------------------------
 static void RunPythonScript(AnsiString scriptPath,AnsiString args);
 static bool DeleteFilesWithExtension(AnsiString dirPath, AnsiString extension);
 static int FinshARTCCBoundary(void);
 //---------------------------------------------------------------------------

static char *stristr(const char *String, const char *Pattern);
static const char * strnistr(const char * pszSource, DWORD dwLength, const char * pszFind) ;

//---------------------------------------------------------------------------
uint32_t createRGB(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
//---------------------------------------------------------------------------
uint32_t PopularColors[] = {
	  createRGB(255, 0, 0),      // Red
	  createRGB(0, 255, 0),      // Green
	  createRGB(0, 0, 255),      // Blue
	  createRGB(255, 255, 0),   // Yellow
	  createRGB(255, 165, 0),   // Orange
	  createRGB(255, 192, 203), // Pink
	  createRGB(0, 255, 255),   // Cyan
	  createRGB(255, 0, 255),  // Magenta
	  createRGB(255,255, 255),    // White
	  //createRGB(0, 0, 0),        // Black
	  createRGB(128,128,128),      // Gray
	  createRGB(165,42,42)    // Brown
  };

  int NumColors = sizeof(PopularColors) / sizeof(PopularColors[0]);
 unsigned int CurrentColor=0;


 //---------------------------------------------------------------------------
 typedef struct
{
   union{ 
     struct{ 
	 System::Byte Red;
	 System::Byte Green;
	 System::Byte Blue;
	 System::Byte Alpha;
     }; 
     struct{ 
     TColor Cl; 
     }; 
     struct{ 
     COLORREF Rgb; 
     }; 
   };

}TMultiColor;

typedef struct
{
  double lat;
  double lon;
  double risk;
}TRiskOverlayCell;

static std::vector<TRiskOverlayCell> g_RiskOverlayCells;
static double g_RiskOverlayCellDeg = 0.01;
static bool g_HaveRiskOverlay = false;

static float RiskToRed(double risk)
{
  if (risk >= 80.0) return 1.0f;
  if (risk >= 60.0) return 1.0f;
  if (risk >= 30.0) return 1.0f;
  return 0.0f;
}

static float RiskToGreen(double risk)
{
  if (risk >= 80.0) return 0.0f;
  if (risk >= 60.0) return 0.55f;
  if (risk >= 30.0) return 0.84f;
  return 0.78f;
}

static float RiskToBlue(double risk)
{
  if (risk >= 80.0) return 0.0f;
  if (risk >= 60.0) return 0.0f;
  if (risk >= 30.0) return 0.0f;
  return 0.33f;
}

static double NormalizeHeading360(double hdg)
{
  while (hdg < 0.0) hdg += 360.0;
  while (hdg >= 360.0) hdg -= 360.0;
  return hdg;
}

static double NormalizeHeadingSignedDiff(double target, double current)
{
  double diff = target - current;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

static AnsiString BuildTrajectorySignature(const std::vector<TGeoPoint> &points)
{
  TFormatSettings fs = TFormatSettings::Create();
  fs.DecimalSeparator = '.';
  AnsiString sig = "";
  double prevLat = 1000.0;
  double prevLon = 1000.0;
  bool havePoint = false;

  for (size_t i = 0; i < points.size(); i++)
  {
    double lat = round(points[i].lat * 100.0) / 100.0;
    double lon = round(points[i].lon * 100.0) / 100.0;

    if (havePoint && fabs(lat - prevLat) < 0.000001 && fabs(lon - prevLon) < 0.000001)
      continue;

    sig += FloatToStrF(lat, ffFixed, 12, 2, fs) + "," + FloatToStrF(lon, ffFixed, 12, 2, fs) + ";";
    prevLat = lat;
    prevLon = lon;
    havePoint = true;
  }
  return sig;
}


//---------------------------------------------------------------------------
static const char * strnistr(const char * pszSource, DWORD dwLength, const char * pszFind)
{
	DWORD        dwIndex   = 0;
	DWORD        dwStrLen  = 0;
	const char * pszSubStr = NULL;

	// check for valid arguments
	if (!pszSource || !pszFind)
	{
		return pszSubStr;
	}

	dwStrLen = strlen(pszFind);

	// can pszSource possibly contain pszFind?
	if (dwStrLen > dwLength)
	{
		return pszSubStr;
	}

	while (dwIndex <= dwLength - dwStrLen)
	{
		if (0 == strnicmp(pszSource + dwIndex, pszFind, dwStrLen))
		{
			pszSubStr = pszSource + dwIndex;
			break;
		}

		dwIndex ++;
	}

	return pszSubStr;
}
//---------------------------------------------------------------------------
static char *stristr(const char *String, const char *Pattern)
{
  char *pptr, *sptr, *start;
  size_t  slen, plen;

  for (start = (char *)String,pptr  = (char *)Pattern,slen  = strlen(String),plen  = strlen(Pattern);
       slen >= plen;start++, slen--)
      {
            /* find start of pattern in string */
            while (toupper(*start) != toupper(*Pattern))
            {
                  start++;
                  slen--;

                  /* if pattern longer than string */

                  if (slen < plen)
                        return(NULL);
            }

            sptr = start;
            pptr = (char *)Pattern;

            while (toupper(*sptr) == toupper(*pptr))
            {
                  sptr++;
                  pptr++;

                  /* if end of pattern then pattern was found */

                  if ('\0' == *pptr)
                        return (start);
            }
      }
   return(NULL);
}
//---------------------------------------------------------------------------
__fastcall TRouteFetchThread::TRouteFetchThread(bool value)
	: TThread(value)
{
	FreeOnTerminate = false;
}
//---------------------------------------------------------------------------
__fastcall TRouteFetchThread::~TRouteFetchThread()
{
}
//---------------------------------------------------------------------------
void __fastcall TRouteFetchThread::DoFetch(void)
{
	if (!Form1) return;

	AnsiString flight = Form1->NormalizeFlightNum(FlightNum);
	if (flight.IsEmpty())
	{
	  Form1->StoreRouteFetchResult(flight,false,"","",false,0.0,0.0);
	  return;
	}

	AnsiString routeText = "";
	AnsiString destCode = "";
	double destLat = 0.0;
	double destLon = 0.0;
	bool success = false;
	bool haveDestination = false;

	TNetHTTPClient *client = new TNetHTTPClient(NULL);
	try
	{
	  _di_IHTTPResponse response;
	  char getStr[1024];
	  snprintf(getStr, sizeof(getStr), API_SERVICE_URL_TXT, flight.c_str(), flight.c_str());
	  response = client->Get(AnsiString(getStr));
	  if (response && response->StatusCode == 200)
	  {
		routeText = Trim(response->ContentAsString(TEncoding::ASCII));
		success = !routeText.IsEmpty();
		if (success && Form1->ParseDestinationCode(routeText, destCode))
		{
		  if (LookupAirportByCode(destCode, destLat, destLon))
			haveDestination = true;
		}
	  }
	}
	catch (...)
	{
	  success = false;
	  routeText = "";
	  destCode = "";
	  haveDestination = false;
	}
	delete client;

	Form1->StoreRouteFetchResult(flight, success, routeText, destCode, haveDestination, destLat, destLon);
}
//---------------------------------------------------------------------------
void __fastcall TRouteFetchThread::Execute(void)
{
	while (!Terminated)
	{
	  if (!Form1)
	  {
		Sleep(100);
		continue;
	  }

	  if (!Form1->DequeueRouteFetch(FlightNum))
	  {
		Sleep(100);
		continue;
	  }

	  try
	  {
		DoFetch();
	  }
	  catch(...)
	  {
		AnsiString flight = Form1->NormalizeFlightNum(FlightNum);
		Form1->StoreRouteFetchResult(flight,false,"","",false,0.0,0.0);
	  }
	}
}
//---------------------------------------------------------------------------
AnsiString __fastcall TForm1::NormalizeCodeToken(const AnsiString &value)
{
	AnsiString out = "";
	const char *ptr = value.c_str();
	if (!ptr) return out;

	for (size_t i = 0; ptr[i] != 0; i++)
	{
	  unsigned char c = (unsigned char)ptr[i];
	  if (isalnum(c))
		out += (char)toupper(c);
	}
	return out;
}
//---------------------------------------------------------------------------
AnsiString __fastcall TForm1::NormalizeFlightNum(const AnsiString &value)
{
	return NormalizeCodeToken(value);
}
//---------------------------------------------------------------------------
bool __fastcall TForm1::DequeueRouteFetch(AnsiString &flightNum)
{
	bool haveData = false;
	RouteCacheCs->Acquire();
	if (!RouteFetchQueue.empty())
	{
	  flightNum = RouteFetchQueue.front();
	  RouteFetchQueue.pop_front();
	  haveData = true;
	}
	RouteCacheCs->Release();
	return haveData;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::StoreRouteFetchResult(const AnsiString &flightNum,
											  bool success,
											  const AnsiString &routeText,
											  const AnsiString &destCode,
											  bool haveDestination,
											  double destLat,
											  double destLon)
{
	if (flightNum.IsEmpty()) return;

	RouteCacheCs->Acquire();
	TRouteCacheEntry &entry = RouteCache[flightNum];
	entry.LastAttemptMs = GetCurrentTimeInMsec();

	if (!success)
	{
	  entry.Status = RFS_FAILED;
	  entry.RouteText = "";
	  entry.DestCode = "";
	  entry.HaveDestination = false;
	  entry.DestLat = 0.0;
	  entry.DestLon = 0.0;
	}
	else
	{
	  entry.Status = RFS_READY;
	  entry.RouteText = routeText;
	  entry.DestCode = destCode;
	  entry.HaveDestination = haveDestination;
	  if (haveDestination)
	  {
		entry.DestLat = destLat;
		entry.DestLon = destLon;
	  }
	  else
	  {
		entry.DestLat = 0.0;
		entry.DestLon = 0.0;
	  }
	}
	RouteCacheCs->Release();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::EnqueueRouteFetch(const AnsiString &flightNum)
{
	AnsiString key = NormalizeFlightNum(flightNum);
	if (key.IsEmpty()) return;

	__int64 now = GetCurrentTimeInMsec();
	bool alreadyQueued = false;

	RouteCacheCs->Acquire();
	TRouteCacheEntry &entry = RouteCache[key];
	if (entry.Status == RFS_PENDING)
	{
	  RouteCacheCs->Release();
	  return;
	}
	if (entry.Status == RFS_READY)
	{
	  RouteCacheCs->Release();
	  return;
	}
	if ((entry.Status == RFS_FAILED) && ((now - entry.LastAttemptMs) < ROUTE_FETCH_RETRY_COOLDOWN_MS))
	{
	  RouteCacheCs->Release();
	  return;
	}
	for (std::deque<AnsiString>::iterator it = RouteFetchQueue.begin(); it != RouteFetchQueue.end(); ++it)
	{
	  if (*it == key)
	  {
		alreadyQueued = true;
		break;
	  }
	}
	if (!alreadyQueued)
	  RouteFetchQueue.push_back(key);

	entry.Status = RFS_PENDING;
	entry.LastAttemptMs = now;
	RouteCacheCs->Release();
}
//---------------------------------------------------------------------------
bool __fastcall TForm1::TryGetCachedRoute(const AnsiString &flightNum, TRouteCacheEntry &entry)
{
	AnsiString key = NormalizeFlightNum(flightNum);
	if (key.IsEmpty()) return false;

	bool haveRoute = false;
	RouteCacheCs->Acquire();
	std::map<AnsiString, TRouteCacheEntry>::iterator it = RouteCache.find(key);
	if (it != RouteCache.end())
	{
	  entry = it->second;
	  haveRoute = true;
	}
	RouteCacheCs->Release();
	return haveRoute;
}
//---------------------------------------------------------------------------
bool __fastcall TForm1::ParseDestinationCode(const AnsiString &routeText, AnsiString &destCode)
{
	destCode = "";
	std::string route = routeText.c_str();
	if (route.empty()) return false;

	for (size_t i = 0; i < route.size(); i++)
	{
	  if (route[i] == '\r' || route[i] == '\n')
		route[i] = ' ';
	}

	std::vector<std::string> tokens;
	size_t start = 0;
	while (true)
	{
	  size_t p = route.find('-', start);
	  if (p == std::string::npos)
	  {
		tokens.push_back(route.substr(start));
		break;
	  }
	  tokens.push_back(route.substr(start, p - start));
	  start = p + 1;
	}

	for (int i = (int)tokens.size() - 1; i >= 0; i--)
	{
	  AnsiString normalized = NormalizeCodeToken(tokens[(size_t)i].c_str());
	  if (!normalized.IsEmpty())
	  {
		destCode = normalized;
		return true;
	  }
	}
	return false;
}
//---------------------------------------------------------------------------
bool __fastcall TForm1::BuildTrajectory(const TADS_B_Aircraft *data,
										const TRouteCacheEntry *routeEntry,
										std::vector<TGeoPoint> &outPoints,
										bool &hasDestination)
{
	outPoints.clear();
	hasDestination = false;

	if (!data || !data->HaveLatLon || !data->HaveSpeedAndHeading || data->Speed <= 0.0)
	  return false;

	TGeoPoint p0;
	p0.lat = data->Latitude;
	p0.lon = data->Longitude;
	outPoints.push_back(p0);

	double simLat = data->Latitude;
	double simLon = data->Longitude;
	double simHdg = NormalizeHeading360(data->Heading);
	const double stepNm1s = data->Speed * (TURN_DT_SECONDS / 3600.0);
	if (stepNm1s <= 0.0)
	  return false;

	if (routeEntry && routeEntry->Status == RFS_READY && routeEntry->HaveDestination)
	{
	  hasDestination = true;

	  for (int t = 0; (t < TURN_PHASE_MAX_SECONDS) && ((int)outPoints.size() < TRAJECTORY_MAX_POINTS); t++)
	  {
		double distNm = 0.0;
		double bearing = 0.0;
		double azBack = 0.0;
		if (VInverse(simLat, simLon, routeEntry->DestLat, routeEntry->DestLon, &distNm, &bearing, &azBack) != OKNOERROR)
		  break;

		if (distNm <= ARRIVAL_THRESHOLD_NM)
		{
		  TGeoPoint pDest;
		  pDest.lat = routeEntry->DestLat;
		  pDest.lon = routeEntry->DestLon;
		  outPoints.push_back(pDest);
		  return outPoints.size() >= 2;
		}

		double err = NormalizeHeadingSignedDiff(bearing, simHdg);
		if (fabs(err) <= HEADING_ALIGN_DEG)
		{
		  break;
		}

		double maxStep = TURN_RATE_DEG_PER_SEC * TURN_DT_SECONDS;
		double delta = err;
		if (delta > maxStep) delta = maxStep;
		if (delta < -maxStep) delta = -maxStep;
		simHdg = NormalizeHeading360(simHdg + delta);

		double nextLat = 0.0;
		double nextLon = 0.0;
		double azTmp = 0.0;
		if (VDirect(simLat, simLon, simHdg, stepNm1s, &nextLat, &nextLon, &azTmp) != OKNOERROR)
		  break;

		simLat = nextLat;
		simLon = nextLon;
		TGeoPoint p;
		p.lat = simLat;
		p.lon = simLon;
		outPoints.push_back(p);
	  }

	  if ((int)outPoints.size() >= TRAJECTORY_MAX_POINTS)
		return outPoints.size() >= 2;

	  double remainNm = 0.0;
	  double az12 = 0.0;
	  double az21 = 0.0;
	  if (VInverse(simLat, simLon, routeEntry->DestLat, routeEntry->DestLon, &remainNm, &az12, &az21) == OKNOERROR)
	  {
		if (remainNm <= ARRIVAL_THRESHOLD_NM)
		{
		  TGeoPoint pDest;
		  pDest.lat = routeEntry->DestLat;
		  pDest.lon = routeEntry->DestLon;
		  outPoints.push_back(pDest);
		  return outPoints.size() >= 2;
		}

		double stepNm = data->Speed * (TRAJECTORY_POINT_SPACING_SECONDS / 3600.0);
		if (stepNm < 1.0) stepNm = 1.0;
		int samples = (int)ceil(remainNm / stepNm);
		if (samples < 1) samples = 1;

		int remainingBudget = TRAJECTORY_MAX_POINTS - (int)outPoints.size();
		if (remainingBudget <= 0)
		  return outPoints.size() >= 2;
		if (samples > remainingBudget)
		  samples = remainingBudget;

		for (int i = 1; i <= samples; i++)
		{
		  double along = remainNm * ((double)i / (double)samples);
		  double lat2 = 0.0;
		  double lon2 = 0.0;
		  double az2 = 0.0;
		  if (VDirect(simLat, simLon, az12, along, &lat2, &lon2, &az2) == OKNOERROR)
		  {
			TGeoPoint p;
			p.lat = lat2;
			p.lon = lon2;
			outPoints.push_back(p);
		  }
		}

		if (!outPoints.empty())
		{
		  outPoints.back().lat = routeEntry->DestLat;
		  outPoints.back().lon = routeEntry->DestLon;
		}
	  }
	  return outPoints.size() >= 2;
	}

	// Fallback trajectory with no destination: simulate forward for 60 seconds.
	int fallbackSteps = (int)TRAJECTORY_LEAD_SECONDS;
	for (int t = 0; (t < fallbackSteps) && ((int)outPoints.size() < TRAJECTORY_MAX_POINTS); t++)
	{
	  double nextLat = 0.0;
	  double nextLon = 0.0;
	  double azTmp = 0.0;
	  if (VDirect(simLat, simLon, simHdg, stepNm1s, &nextLat, &nextLon, &azTmp) != OKNOERROR)
		break;

	  simLat = nextLat;
	  simLon = nextLon;
	  TGeoPoint p;
	  p.lat = simLat;
	  p.lon = simLon;
	  outPoints.push_back(p);
	}

	return outPoints.size() >= 2;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::DrawTrajectory(const std::vector<TGeoPoint> &points, bool hasDestination)
{
	if (points.size() < 2) return;

	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
	glColor4f(1.0, 0.0, 0.0, 1.0);
	glDisable(GL_LINE_STIPPLE);
	glLineWidth(4.0);

	bool stripOpen = false;
	for (size_t i = 0; i < points.size(); i++)
	{
	  if ((i > 0) && (fabs(points[i - 1].lon - points[i].lon) > 180.0))
	  {
		if (stripOpen)
		{
		  glEnd();
		  stripOpen = false;
		}
	  }

	  if (!stripOpen)
	  {
		glBegin(GL_LINE_STRIP);
		stripOpen = true;
	  }

	  double x = 0.0;
	  double y = 0.0;
	  LatLon2XY(points[i].lat, points[i].lon, x, y);
	  glVertex2f(x, y);
	}
	if (stripOpen) glEnd();

	if (hasDestination)
	{
	  const TGeoPoint &dest = points.back();
	  double x = 0.0;
	  double y = 0.0;
	  LatLon2XY(dest.lat, dest.lon, x, y);

	  glDisable(GL_LINE_STIPPLE);
	  glLineWidth(3.0);
	  const float radius = 8.0f;
	  glBegin(GL_LINE_LOOP);
	  for (int i = 0; i < 20; i++)
	  {
		float ang = (float)(2.0 * M_PI * (double)i / 20.0);
		glVertex2f((float)x + cosf(ang) * radius, (float)y + sinf(ang) * radius);
	  }
	  glEnd();

	  glBegin(GL_LINES);
	  glVertex2f((float)x - radius, (float)y);
	  glVertex2f((float)x + radius, (float)y);
	  glVertex2f((float)x, (float)y - radius);
	  glVertex2f((float)x, (float)y + radius);
	  glEnd();
	}
	glPopAttrib();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ClearTrajectoryState()
{
	TrajectoryState.Valid = false;
	TrajectoryState.ICAO = 0;
	TrajectoryState.Points.clear();
	TrajectoryState.LastLat = 0.0;
	TrajectoryState.LastLon = 0.0;
	TrajectoryState.LastHeading = 0.0;
	TrajectoryState.LastSpeed = 0.0;
	TrajectoryState.LastRouteKey = "";
	TrajectoryState.HadDestination = false;
	LastWeatherOverlaySignature = "";
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
  AircraftDBPathFileName=ExtractFilePath(ExtractFileDir(Application->ExeName)) +AnsiString("..\\AircraftDB\\")+AIRCRAFT_DATABASE_FILE;
  ARTCCBoundaryDataPathFileName=ExtractFilePath(ExtractFileDir(Application->ExeName)) +AnsiString("..\\ARTCC_Boundary_Data\\")+ARTCC_BOUNDARY_FILE;
  BigQueryPath=ExtractFilePath(ExtractFileDir(Application->ExeName)) +AnsiString("..\\BigQuery\\");
  BigQueryPythonScript= BigQueryPath+ AnsiString(BIG_QUERY_RUN_FILENAME);
  DeleteFilesWithExtension(BigQueryPath, "csv");
  BigQueryLogFileName=BigQueryPath+"BigQuery.log";
  DeleteFileA(BigQueryLogFileName.c_str());
  CurrentSpriteImage=0;
  InitDecodeRawADS_B();
  RecordRawStream=NULL;
  PlayBackRawStream=NULL;
  RouteCacheCs = new System::Syncobjs::TCriticalSection();
  RouteFetchThread = NULL;
  LastWeatherOverlayPostMs = 0;
  LastWeatherOverlaySignature = "";
  TrackHook.Valid_CC=false;
  TrackHook.Valid_CPA=false;
  ClearTrajectoryState();

  HashTable = ght_create(50000);

  if ( !HashTable)
	{
	  throw Sysutils::Exception("Create Hash Failed");
	}
  ght_set_rehash(HashTable, TRUE);

  AreaTemp=NULL;
  Areas= new TList;

 MouseDown=false;

 MapCenterLat=MAP_CENTER_LAT;
 MapCenterLon=MAP_CENTER_LON;

 LoadMapFromInternet=true;
 MapComboBox->ItemIndex=GoogleMaps;
 //MapComboBox->ItemIndex=SkyVector_VFR;
 //MapComboBox->ItemIndex=SkyVector_IFR_Low;
 //MapComboBox->ItemIndex=SkyVector_IFR_High;
 LoadMap(MapComboBox->ItemIndex);

 g_EarthView->m_Eye.h /= pow(1.3,18);//pow(1.3,43);
 SetMapCenter(g_EarthView->m_Eye.x, g_EarthView->m_Eye.y);
 TimeToGoTrackBar->Position=120;
  BigQueryCSV=NULL;
  BigQueryRowCount=0;
  BigQueryFileCount=0;
  InitAircraftDB(AircraftDBPathFileName);
  AirportLookupPathFileName=ExtractFilePath(ExtractFileDir(Application->ExeName)) +AnsiString("..\\AirportDB\\airport_lookup.csv");
  if (!InitAirportLookup(AirportLookupPathFileName))
  {
	printf("Warning: Failed to load airport lookup file %s\n", AirportLookupPathFileName.c_str());
  }
  Form1->SpVoice1->Rate=2; // Set Rate of Voice
  Form1->SpVoice1->Volume=100;  //Set Volume of Voice
 
 NetHTTPClientPrediction= new TNetHTTPClient(this);
 NetHTTPClientPrediction->OnRequestCompleted=NetHTTPClientPredictionRequestCompleted;
 NetHTTPClientPrediction->OnRequestError=NetHTTPClientPredictionRequestError;
 NetHTTPClientWeather= new TNetHTTPClient(this);
 NetHTTPClientWeather->OnRequestCompleted=NetHTTPClientWeatherRequestCompleted;
 NetHTTPClientWeather->OnRequestError=NetHTTPClientWeatherRequestError;

 PhaseLabel= new TLabel(Panel4);
 PhaseLabel->Parent=Panel4;
 PhaseLabel->Left=5;
 PhaseLabel->Top=RouteLabel->Top + RouteLabel->Height + 4;
 PhaseLabel->Caption="Phase: N/A";
 PhaseLabel->Font->Assign(RouteLabel->Font);
 PhaseLabel->AutoSize=true;
 PhaseLabel->BringToFront();

 // Make room for phase line under ROUTE in the Close Control panel.
 {
   const int delta = 22;
   const int panel4Bottom = Panel4->Top + Panel4->Height;
   Panel4->Height += delta;
   for (int i = 0; i < Panel3->ControlCount; i++) {
     TControl *c = Panel3->Controls[i];
     if (c != Panel4 && c->Top >= panel4Bottom) {
       c->Top += delta;
     }
   }
	 }

	 RouteFetchThread = new TRouteFetchThread(true);
	 RouteFetchThread->FreeOnTerminate = false;
	 RouteFetchThread->Resume();

	 printf("init complete\n");
}
//---------------------------------------------------------------------------
__fastcall TForm1::~TForm1()
{
 Timer1->Enabled=false;
 Timer2->Enabled=false;
 if (RouteFetchThread)
 {
   RouteFetchThread->Terminate();
   RouteFetchThread->WaitFor();
   delete RouteFetchThread;
   RouteFetchThread = NULL;
 }
 if (RouteCacheCs)
 {
   delete RouteCacheCs;
   RouteCacheCs = NULL;
 }
 delete g_EarthView;
 if (g_GETileManager) delete g_GETileManager;
 delete g_MasterLayer;
 delete g_Storage;
 if (LoadMapFromInternet)
 {
   if (g_Keyhole) delete g_Keyhole;
 }

}
//---------------------------------------------------------------------------
void __fastcall  TForm1::SetMapCenter(double &x, double &y)
{
  double siny;
  x=(MapCenterLon+0.0)/360.0;
  siny=sin((MapCenterLat * M_PI) / 180.0);
  siny = fmin(fmax(siny, -0.9999), 0.9999);
  y=(log((1 + siny) / (1 - siny)) / (4 * M_PI));
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ObjectDisplayInit(TObject *Sender)
{
	glViewport(0,0,(GLsizei)ObjectDisplay->Width,(GLsizei)ObjectDisplay->Height);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable (GL_LINE_STIPPLE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    NumSpriteImages=MakeAirplaneImages();
	MakeAirTrackFriend();
	MakeAirTrackHostile();
	MakeAirTrackUnknown();
	MakePoint();
	MakeTrackHook();
	g_EarthView->Resize(ObjectDisplay->Width,ObjectDisplay->Height);
	glPushAttrib (GL_LINE_BIT);
	glPopAttrib ();
    printf("OpenGL Version %s\n",glGetString(GL_VERSION));
}
//---------------------------------------------------------------------------

void __fastcall TForm1::ObjectDisplayResize(TObject *Sender)
{
	 double Value;
	//ObjectDisplay->Width=ObjectDisplay->Height;
	glViewport(0,0,(GLsizei)ObjectDisplay->Width,(GLsizei)ObjectDisplay->Height);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable (GL_LINE_STIPPLE);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	g_EarthView->Resize(ObjectDisplay->Width,ObjectDisplay->Height);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ObjectDisplayPaint(TObject *Sender)
{

 if (DrawMap->Checked)glClearColor(0.0,0.0,0.0,0.0);
 else	glClearColor(BG_INTENSITY,BG_INTENSITY,BG_INTENSITY,0.0);

 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

 g_EarthView->Animate();
 g_EarthView->Render(DrawMap->Checked);
 g_GETileManager->Cleanup();
 Mw1 = Map_w[1].x-Map_w[0].x;
 Mw2 = Map_v[1].x-Map_v[0].x;
 Mh1 = Map_w[1].y-Map_w[0].y;
 Mh2 = Map_v[3].y-Map_v[0].y;

 xf=Mw1/Mw2;
 yf=Mh1/Mh2;

 DrawObjects();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Timer1Timer(TObject *Sender)
{
 __int64 CurrentTime;

 CurrentTime=GetCurrentTimeInMsec();
 SystemTime->Caption=TimeToChar(CurrentTime);

 ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::DrawObjects(void)
{
  double ScrX, ScrY;
  int    ViewableAircraft=0;

  glEnable( GL_LINE_SMOOTH );
  glEnable( GL_POINT_SMOOTH );
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
  glLineWidth(3.0);
  glPointSize(4.0);
  glColor4f(1.0, 1.0, 1.0, 1.0);

  LatLon2XY(MapCenterLat,MapCenterLon, ScrX, ScrY);

  glBegin(GL_LINE_STRIP);
  glVertex2f(ScrX-20.0,ScrY);
  glVertex2f(ScrX+20.0,ScrY);
  glEnd();

  glBegin(GL_LINE_STRIP);
  glVertex2f(ScrX,ScrY-20.0);
  glVertex2f(ScrX,ScrY+20.0);
  glEnd();

  if (g_HaveRiskOverlay && g_RiskOverlayCellDeg > 0.0)
  {
    const double half = g_RiskOverlayCellDeg * 0.5;
    glLineWidth(1.0);
    for (size_t ri = 0; ri < g_RiskOverlayCells.size(); ri++)
    {
      const TRiskOverlayCell &rc = g_RiskOverlayCells[ri];
      double x1, y1, x2, y2, x3, y3, x4, y4;
      LatLon2XY(rc.lat - half, rc.lon - half, x1, y1);
      LatLon2XY(rc.lat - half, rc.lon + half, x2, y2);
      LatLon2XY(rc.lat + half, rc.lon + half, x3, y3);
      LatLon2XY(rc.lat + half, rc.lon - half, x4, y4);

      glColor4f(RiskToRed(rc.risk), RiskToGreen(rc.risk), RiskToBlue(rc.risk), 0.32f);
      glBegin(GL_TRIANGLES);
      glVertex2f(x1, y1);
      glVertex2f(x2, y2);
      glVertex2f(x3, y3);
      glVertex2f(x1, y1);
      glVertex2f(x3, y3);
      glVertex2f(x4, y4);
      glEnd();
    }
  }


  uint32_t *Key;
  ght_iterator_t iterator;
  TADS_B_Aircraft* Data,*DataCPA;

  DWORD i,j,Count;

  if (AreaTemp)
  {
   glPointSize(3.0);
	for (DWORD i = 0; i <AreaTemp->NumPoints ; i++)
		LatLon2XY(AreaTemp->Points[i][1],AreaTemp->Points[i][0],
				  AreaTemp->PointsAdj[i][0],AreaTemp->PointsAdj[i][1]);

   glBegin(GL_POINTS);
   for (DWORD i = 0; i <AreaTemp->NumPoints ; i++)
	{
	glVertex2f(AreaTemp->PointsAdj[i][0],
			   AreaTemp->PointsAdj[i][1]);
	}
	glEnd();
   glBegin(GL_LINE_STRIP);
   for (DWORD i = 0; i <AreaTemp->NumPoints ; i++)
	{
	glVertex2f(AreaTemp->PointsAdj[i][0],
			   AreaTemp->PointsAdj[i][1]);
	}
	glEnd();
  }
	Count=Areas->Count;
	for (i = 0; i < Count; i++)
	 {
	   TArea *Area = (TArea *)Areas->Items[i];
	   TMultiColor MC;

	   MC.Rgb=ColorToRGB(Area->Color);
	   if (Area->Selected)
	   {
		  glLineWidth(4.0);
		  glPushAttrib (GL_LINE_BIT);
		  glLineStipple (3, 0xAAAA);
	   }


	   glColor4f(MC.Red/255.0, MC.Green/255.0, MC.Blue/255.0, 1.0);
	   glBegin(GL_LINE_LOOP);
	   for (j = 0; j <Area->NumPoints; j++)
	   {
		LatLon2XY(Area->Points[j][1],Area->Points[j][0], ScrX, ScrY);
		glVertex2f(ScrX,ScrY);
	   }
	  glEnd();
	   if (Area->Selected)
	   {
		glPopAttrib ();
		glLineWidth(2.0);
	   }

	   glColor4f(MC.Red/255.0, MC.Green/255.0, MC.Blue/255.0, 0.4);

	   for (j = 0; j <Area->NumPoints; j++)
	   {
		LatLon2XY(Area->Points[j][1],Area->Points[j][0],
				  Area->PointsAdj[j][0],Area->PointsAdj[j][1]);
	   }
	  TTriangles *Tri=Area->Triangles;

	  while(Tri)
	  {
		glBegin(GL_TRIANGLES);
		glVertex2f(Area->PointsAdj[Tri->indexList[0]][0],
				   Area->PointsAdj[Tri->indexList[0]][1]);
		glVertex2f(Area->PointsAdj[Tri->indexList[1]][0],
				   Area->PointsAdj[Tri->indexList[1]][1]);
		glVertex2f(Area->PointsAdj[Tri->indexList[2]][0],
				   Area->PointsAdj[Tri->indexList[2]][1]);
		glEnd();
		Tri=Tri->next;
	  }
	 }

    AircraftCountLabel->Caption=IntToStr((int)ght_size(HashTable));
	for(Data = (TADS_B_Aircraft *)ght_first(HashTable, &iterator,(const void **) &Key);
			  Data; Data = (TADS_B_Aircraft *)ght_next(HashTable, &iterator, (const void **)&Key))
	{
	  if (Data->HaveLatLon)
	  {
		ViewableAircraft++;
	   glColor4f(1.0, 1.0, 1.0, 1.0);

	   LatLon2XY(Data->Latitude,Data->Longitude, ScrX, ScrY);
	   //DrawPoint(ScrX,ScrY);
	   if (Data->HaveSpeedAndHeading)   glColor4f(1.0, 0.0, 1.0, 1.0);
	   else
		{
		 Data->Heading=0.0;
		 glColor4f(1.0, 0.0, 0.0, 1.0);
		}

	   DrawAirplaneImage(ScrX,ScrY,1.5,Data->Heading,Data->SpriteImage);
	   glRasterPos2i(ScrX+30,ScrY-10);
	   ObjectDisplay->Draw2DText(Data->HexAddr);

	   if ((Data->HaveSpeedAndHeading) && (TimeToGoCheckBox->State==cbChecked))
	   {
		double lat,lon,az;
		if (VDirect(Data->Latitude,Data->Longitude,
					Data->Heading,Data->Speed/3060.0*TimeToGoTrackBar->Position ,&lat,&lon,&az)==OKNOERROR)
		  {
			 double ScrX2, ScrY2;
			 LatLon2XY(lat,lon, ScrX2, ScrY2);
             glColor4f(1.0, 1.0, 0.0, 1.0);
			 glBegin(GL_LINE_STRIP);
			 glVertex2f(ScrX,ScrY);
			 glVertex2f(ScrX2,ScrY2);
			 glEnd();
		  }
	   }
	 }
	}
 ViewableAircraftCountLabel->Caption=ViewableAircraft;
 if (TrackHook.Valid_CC)
 {
		Data= (TADS_B_Aircraft *)ght_get(HashTable, sizeof(TrackHook.ICAO_CC), (void *)&TrackHook.ICAO_CC);
		if (Data)
		{
		TRouteCacheEntry routeEntry;
		bool haveRouteEntry = false;
		bool routeReady = false;
		AnsiString routeStateKey = "LEAD_ONLY";

		ICAOLabel->Caption=Data->HexAddr;
		if (Data->HaveFlightNum)
		  {
		   FlightNumLabel->Caption=Data->FlightNum;
		   AnsiString flightNum = NormalizeFlightNum(Data->FlightNum);
		   if (!flightNum.IsEmpty())
		   {
			 haveRouteEntry = TryGetCachedRoute(flightNum, routeEntry);
			 if (!haveRouteEntry)
			 {
			   Data->HaveRoute = false;
			   EnqueueRouteFetch(flightNum);
			   RouteLabel->Caption="LOADING";
			 }
			 else
			 {
			   if (routeEntry.Status == RFS_READY)
			   {
				 routeReady = true;
				 Data->HaveRoute = false;
				 if (strlen(routeEntry.RouteText.c_str()) < sizeof(Data->Route))
				 {
				   strcpy(Data->Route, routeEntry.RouteText.c_str());
				   Data->HaveRoute = true;
				 }
				 RouteLabel->Caption = routeEntry.RouteText;
				 routeStateKey = routeEntry.RouteText + "|" + routeEntry.DestCode;
			   }
			   else if (routeEntry.Status == RFS_PENDING)
			   {
				 Data->HaveRoute = false;
				 RouteLabel->Caption="LOADING";
			   }
			   else if (routeEntry.Status == RFS_FAILED)
			   {
				 Data->HaveRoute = false;
				 RouteLabel->Caption="UNKNOWN";
				 if ((GetCurrentTimeInMsec() - routeEntry.LastAttemptMs) >= ROUTE_FETCH_RETRY_COOLDOWN_MS)
				   EnqueueRouteFetch(flightNum);
			   }
			   else
			   {
				 Data->HaveRoute = false;
				 EnqueueRouteFetch(flightNum);
				 RouteLabel->Caption="LOADING";
			   }
			 }
		   }
		   else
		   {
			 Data->HaveRoute = false;
			 RouteLabel->Caption="UNKNOWN";
		   }
		  }
		else
		{
		  FlightNumLabel->Caption="N/A";
		  Data->HaveRoute = false;
		  RouteLabel->Caption="N/A";
		}
        if (Data->HaveLatLon)
		{
		 CLatLabel->Caption=DMS::DegreesMinutesSecondsLat(Data->Latitude).c_str();
		 CLonLabel->Caption=DMS::DegreesMinutesSecondsLon(Data->Longitude).c_str();
        }
        else
        {
         CLatLabel->Caption="N/A";
		 CLonLabel->Caption="N/A";
        }
        if (Data->HaveSpeedAndHeading)
        {
		 SpdLabel->Caption=FloatToStrF(Data->Speed, ffFixed,12,2)+" KTS  VRATE:"+FloatToStrF(Data->VerticalRate,ffFixed,12,2);
		 HdgLabel->Caption=FloatToStrF(Data->Heading, ffFixed,12,2)+" DEG";
        }
        else
        {
 		 SpdLabel->Caption="N/A";
		 HdgLabel->Caption="N/A";
        }
        if (Data->Altitude)
		 AltLabel->Caption= FloatToStrF(Data->Altitude, ffFixed,12,2)+" FT";
		else AltLabel->Caption="N/A";

		MsgCntLabel->Caption="Raw: "+IntToStr((int)Data->NumMessagesRaw)+" SBS: "+IntToStr((int)Data->NumMessagesSBS);
		TrkLastUpdateTimeLabel->Caption=TimeToChar(Data->LastSeen);

        glColor4f(1.0, 0.0, 0.0, 1.0);
        LatLon2XY(Data->Latitude,Data->Longitude, ScrX, ScrY);
        DrawTrackHook(ScrX, ScrY);

		if (Data->HaveLatLon && Data->HaveSpeedAndHeading && (Data->Speed > 0.0))
		{
		  bool trajectoryChanged = (!TrajectoryState.Valid) ||
								   (TrajectoryState.ICAO != Data->ICAO) ||
								   (fabs(TrajectoryState.LastLat - Data->Latitude) > 0.000001) ||
								   (fabs(TrajectoryState.LastLon - Data->Longitude) > 0.000001) ||
								   (fabs(TrajectoryState.LastHeading - Data->Heading) > 0.000001) ||
								   (fabs(TrajectoryState.LastSpeed - Data->Speed) > 0.000001) ||
								   (TrajectoryState.LastRouteKey != routeStateKey);
		  if (trajectoryChanged)
		  {
			std::vector<TGeoPoint> points;
			bool hasDestination = false;
			const TRouteCacheEntry *routePtr = routeReady ? &routeEntry : NULL;

			if (BuildTrajectory(Data, routePtr, points, hasDestination))
			{
			  TrajectoryState.Valid = true;
			  TrajectoryState.ICAO = Data->ICAO;
			  TrajectoryState.Points = points;
			  TrajectoryState.LastLat = Data->Latitude;
			  TrajectoryState.LastLon = Data->Longitude;
			  TrajectoryState.LastHeading = Data->Heading;
			  TrajectoryState.LastSpeed = Data->Speed;
			  TrajectoryState.LastRouteKey = routeStateKey;
			  TrajectoryState.HadDestination = hasDestination;

			  const AnsiString signature = BuildTrajectorySignature(TrajectoryState.Points);
			  const __int64 nowMs = GetCurrentTimeInMsec();
			  if (!signature.IsEmpty() &&
				  (signature != LastWeatherOverlaySignature) &&
				  ((LastWeatherOverlayPostMs == 0) || ((nowMs - LastWeatherOverlayPostMs) >= WEATHER_POST_THROTTLE_MS)))
			  {
				SendPredictedRoutePointsToBackendFromPoints(TrajectoryState.Points);
				LastWeatherOverlayPostMs = nowMs;
				LastWeatherOverlaySignature = signature;
			  }
			}
			else ClearTrajectoryState();
		  }
		  if (TrajectoryState.Valid)
			DrawTrajectory(TrajectoryState.Points, TrajectoryState.HadDestination);
		}
		else ClearTrajectoryState();
        }

		else
        {
		 TrackHook.Valid_CC=false;
		 ClearTrajectoryState();
		 ICAOLabel->Caption="N/A";
		 FlightNumLabel->Caption="N/A";
		 RouteLabel->Caption="N/A";
         CLatLabel->Caption="N/A";
		 CLonLabel->Caption="N/A";
         SpdLabel->Caption="N/A";
		 HdgLabel->Caption="N/A";
		 AltLabel->Caption="N/A";
		 MsgCntLabel->Caption="N/A";
         TrkLastUpdateTimeLabel->Caption="N/A";
        }
 }
 else
 {
   ClearTrajectoryState();
   RouteLabel->Caption="N/A";
 }
 if (TrackHook.Valid_CPA)
 {
  bool CpaDataIsValid=false;
  DataCPA= (TADS_B_Aircraft *)ght_get(HashTable, sizeof(TrackHook.ICAO_CPA), (void *)&TrackHook.ICAO_CPA);
  if ((DataCPA) && (TrackHook.Valid_CC))
	{

	  double tcpa,cpa_distance_nm, vertical_cpa;
	  double lat1, lon1,lat2, lon2, junk;
	  if (computeCPA(Data->Latitude, Data->Longitude, Data->Altitude,
					 Data->Speed,Data->Heading,
					 DataCPA->Latitude, DataCPA->Longitude, DataCPA->Altitude,
					 DataCPA->Speed,DataCPA->Heading,
					 tcpa,cpa_distance_nm, vertical_cpa))
	  {
		if (VDirect(Data->Latitude,Data->Longitude,
					Data->Heading,Data->Speed/3600.0*tcpa,&lat1,&lon1,&junk)==OKNOERROR)
		{
		  if (VDirect(DataCPA->Latitude,DataCPA->Longitude,
					  DataCPA->Heading,DataCPA->Speed/3600.0*tcpa,&lat2,&lon2,&junk)==OKNOERROR)
		   {
			 glColor4f(0.0, 1.0, 0.0, 1.0);
			 glBegin(GL_LINE_STRIP);
			 LatLon2XY(Data->Latitude,Data->Longitude, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 LatLon2XY(lat1,lon1, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 glEnd();
			 glBegin(GL_LINE_STRIP);
			 LatLon2XY(DataCPA->Latitude,DataCPA->Longitude, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 LatLon2XY(lat2,lon2, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 glEnd();
			 glColor4f(1.0, 0.0, 0.0, 1.0);
			 glBegin(GL_LINE_STRIP);
			 LatLon2XY(lat1,lon1, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 LatLon2XY(lat2,lon2, ScrX, ScrY);
			 glVertex2f(ScrX, ScrY);
			 glEnd();
			 CpaTimeValue->Caption=TimeToChar(tcpa*1000);
			 CpaDistanceValue->Caption= FloatToStrF(cpa_distance_nm, ffFixed,10,2)+" NM VDIST: "+IntToStr((int)vertical_cpa)+" FT";
			 CpaDataIsValid=true;
		   }
		}
	  }
	}
   if (!CpaDataIsValid)
   {
	TrackHook.Valid_CPA=false;
	CpaTimeValue->Caption="None";
	CpaDistanceValue->Caption="None";
   }
 }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ObjectDisplayMouseDown(TObject *Sender,
	  TMouseButton Button, TShiftState Shift, int X, int Y)
{

 if (Button==mbLeft)
   {
	if (Shift.Contains(ssCtrl))
	{

	}
	else
	{
	 g_MouseLeftDownX = X;
	 g_MouseLeftDownY = Y;
	 g_MouseDownMask |= LEFT_MOUSE_DOWN ;
	 g_EarthView->StartDrag(X, Y, NAV_DRAG_PAN);
	}
  }
 else if (Button==mbRight)
  {
  if (AreaTemp)
   {
	if (AreaTemp->NumPoints<MAX_AREA_POINTS)
	{
	  AddPoint(X, Y);
	}
	else ShowMessage("Max Area Points Reached");
   }
  else
   {
   if (Shift.Contains(ssCtrl))   HookTrack(X,Y,true);
   else  HookTrack(X,Y,false);
   }
  }

 else if (Button==mbMiddle)  ResetXYOffset();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::ObjectDisplayMouseUp(TObject *Sender,
	  TMouseButton Button, TShiftState Shift, int X, int Y)
{
  if (Button == mbLeft) g_MouseDownMask &= ~LEFT_MOUSE_DOWN;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ObjectDisplayMouseMove(TObject *Sender,
	  TShiftState Shift, int X, int Y)
{
 int X1,Y1;
 double VLat,VLon;
 int i;
 Y1=(ObjectDisplay->Height-1)-Y;
 X1=X;
 if  ((X1>=Map_v[0].x) && (X1<=Map_v[1].x) &&
	  (Y1>=Map_v[0].y) && (Y1<=Map_v[3].y))

  {
   pfVec3 Point;
   VLat=atan(sinh(M_PI * (2 * (Map_w[1].y-(yf*(Map_v[3].y-Y1))))))*(180.0 / M_PI);
   VLon=(Map_w[1].x-(xf*(Map_v[1].x-X1)))*360.0;
   Lat->Caption=DMS::DegreesMinutesSecondsLat(VLat).c_str();
   Lon->Caption=DMS::DegreesMinutesSecondsLon(VLon).c_str();
   Point[0]=VLon;
   Point[1]=VLat;
   Point[2]=0.0;

   for (i = 0; i < Areas->Count; i++)
	 {
	   TArea *Area = (TArea *)Areas->Items[i];
	   if (PointInPolygon(Area->Points,Area->NumPoints,Point))
	   {
#if 0
		  MsgLog->Lines->Add("In Polygon "+ Area->Name);
#endif
       }
	 }
  }

  if (g_MouseDownMask & LEFT_MOUSE_DOWN)
  {
   g_EarthView->Drag(g_MouseLeftDownX, g_MouseLeftDownY, X,Y, NAV_DRAG_PAN);
   ObjectDisplay->Repaint();
  }

}
//---------------------------------------------------------------------------
void __fastcall TForm1::ResetXYOffset(void)
{
 SetMapCenter(g_EarthView->m_Eye.x, g_EarthView->m_Eye.y);
 ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Exit1Click(TObject *Sender)
{
 Close();
}
//---------------------------------------------------------------------------
 void __fastcall TForm1::AddPoint(int X, int Y)
 {
  double Lat,Lon;

 if (XY2LatLon2(X,Y,Lat,Lon)==0)
 {

	AreaTemp->Points[AreaTemp->NumPoints][1]=Lat;
	AreaTemp->Points[AreaTemp->NumPoints][0]=Lon;
	AreaTemp->Points[AreaTemp->NumPoints][2]=0.0;
	AreaTemp->NumPoints++;
	ObjectDisplay->Repaint();
 }
 }
 //---------------------------------------------------------------------------

wchar_t *AnsiTowchar_t(AnsiString Str)
{
wchar_t *str = new wchar_t[Str.WideCharBufSize()];
return Str.WideChar(str, Str.WideCharBufSize());
}
//---------------------------------------------------------------------------
AnsiString __fastcall TForm1::BuildPredictedRoutePointsJsonFromPoints(const std::vector<TGeoPoint> &pointsIn)
{
 if (pointsIn.size() < 2) return "";

 TFormatSettings fs = TFormatSettings::Create();
 fs.DecimalSeparator = '.';
 double prevLat = 1000.0;
 double prevLon = 1000.0;
 bool havePoint = false;
 int uniqueCount = 0;

 AnsiString points = "";
 for (size_t i = 0; i < pointsIn.size(); i++)
 {
   double lat = round(pointsIn[i].lat * 100.0) / 100.0;
   double lon = round(pointsIn[i].lon * 100.0) / 100.0;

   if (havePoint && fabs(lat - prevLat) < 0.000001 && fabs(lon - prevLon) < 0.000001)
	 continue;

   if (points.Length() > 0) points += ",";
   points += "{\"lat\":" + FloatToStrF(lat, ffFixed, 12, 2, fs) + ",\"lon\":" + FloatToStrF(lon, ffFixed, 12, 2, fs) + "}";
   prevLat = lat;
   prevLon = lon;
   havePoint = true;
   uniqueCount++;
 }

 if (uniqueCount < 2) return "";

 AnsiString json =
   "{"
   "\"trajectory\":[" + points + "],"
   "\"hourly\":["
   "\"precipitation\","
   "\"rain\","
   "\"snowfall\","
   "\"precipitation_probability\","
   "\"weather_code\","
   "\"cape\","
   "\"wind_speed_10m\","
   "\"wind_gusts_10m\","
   "\"wind_direction_10m\","
   "\"visibility\","
   "\"temperature_2m\""
   "],"
   "\"forecast_days\":1,"
   "\"cell_deg\":0.01,"
   "\"densify\":false,"
   "\"radius_cells\":0,"
   "\"time_mode\":\"nearest\","
   "\"include_series\":false,"
   "\"risk_profile\":\"safety\","
   "\"include_risk_details\":false,"
   "\"coord_mode\":\"input_point\""
   "}";

 return json;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::SendPredictedRoutePointsToBackendFromPoints(const std::vector<TGeoPoint> &points)
{
 AnsiString payload = BuildPredictedRoutePointsJsonFromPoints(points);
 if (payload.IsEmpty()) return;

 TStringStream *body = NULL;
 try
 {
  body = new TStringStream((UnicodeString)payload, TEncoding::UTF8, false);
  System::Net::Urlclient::TNetHeaders headers;
  headers.Length = 1;
  headers[0] = System::Net::Urlclient::TNameValuePair("Content-Type", "application/json");

  _di_IHTTPResponse response = NetHTTPClientWeather->Post(AnsiString(WEATHER_OVERLAY_URL), body, NULL, headers);
  if (!response || (response->StatusCode < 200 || response->StatusCode >= 300))
  {
	// Non-fatal: weather overlay is optional.
	return;
  }

  UnicodeString content = response->ContentAsString(TEncoding::UTF8);
  std::unique_ptr<TJSONValue> root(TJSONObject::ParseJSONValue(content));
  if (!root) return;

  TJSONObject *obj = dynamic_cast<TJSONObject *>(root.get());
  if (!obj) return;

  TJSONValue *cellDegValue = obj->GetValue("cell_deg");
  TFormatSettings fs = TFormatSettings::Create();
  fs.DecimalSeparator = '.';
  if (cellDegValue)
  {
	try
	{
	  g_RiskOverlayCellDeg = StrToFloat((UnicodeString)cellDegValue->Value(), fs);
	}
	catch(...)
	{
	  g_RiskOverlayCellDeg = 0.01;
	}
  }

  TJSONValue *cellsValue = obj->GetValue("cells");
  TJSONArray *cells = dynamic_cast<TJSONArray *>(cellsValue);
  if (!cells) return;

  g_RiskOverlayCells.clear();
  for (int i = 0; i < cells->Count; i++)
  {
	TJSONObject *cellObj = dynamic_cast<TJSONObject *>(cells->Items[i]);
	if (!cellObj) continue;

	TJSONValue *latValue = cellObj->GetValue("cell_lat");
	TJSONValue *lonValue = cellObj->GetValue("cell_lon");
	TJSONValue *riskValue = cellObj->GetValue("risk_score");
	if (!latValue || !lonValue || !riskValue) continue;

	TRiskOverlayCell c;
	try
	{
	  c.lat = StrToFloat((UnicodeString)latValue->Value(), fs);
	  c.lon = StrToFloat((UnicodeString)lonValue->Value(), fs);
	  c.risk = StrToFloat((UnicodeString)riskValue->Value(), fs);
	  g_RiskOverlayCells.push_back(c);
	}
	catch(...)
	{
	  continue;
	}
  }

  g_HaveRiskOverlay = !g_RiskOverlayCells.empty();
 }
 catch(...)
 {
   // Non-fatal: keep UI behavior unchanged if backend is unavailable.
 }

 if (body) delete body;
}
//---------------------------------------------------------------------------
AnsiString __fastcall TForm1::BuildPredictedRoutePointsJson(TADS_B_Aircraft *Data)
{
 if (!Data) return "";

 if (TrajectoryState.Valid && (TrajectoryState.ICAO == Data->ICAO) && !TrajectoryState.Points.empty())
   return BuildPredictedRoutePointsJsonFromPoints(TrajectoryState.Points);

 std::vector<TGeoPoint> fallbackPoints;
 bool hadDestination = false;
 if (BuildTrajectory(Data, NULL, fallbackPoints, hadDestination))
   return BuildPredictedRoutePointsJsonFromPoints(fallbackPoints);

 return "";
}
//---------------------------------------------------------------------------
void __fastcall TForm1::SendPredictedRoutePointsToBackend(TADS_B_Aircraft *Data)
{
 if (!Data) return;

 if (TrajectoryState.Valid && (TrajectoryState.ICAO == Data->ICAO) && !TrajectoryState.Points.empty())
 {
   SendPredictedRoutePointsToBackendFromPoints(TrajectoryState.Points);
   return;
 }

 std::vector<TGeoPoint> fallbackPoints;
 bool hadDestination = false;
 if (BuildTrajectory(Data, NULL, fallbackPoints, hadDestination))
   SendPredictedRoutePointsToBackendFromPoints(fallbackPoints);
}
//---------------------------------------------------------------------------
 void __fastcall TForm1::HookTrack(int X, int Y,bool CPA_Hook)
 {
  double VLat,VLon, dlat,dlon,Range;
  int X1,Y1;
   uint32_t *Key;

   uint32_t Current_ICAO;
   double MinRange;
  ght_iterator_t iterator;
  TADS_B_Aircraft* Data;

  Y1=(ObjectDisplay->Height-1)-Y;
  X1=X;

  if  ((X1<Map_v[0].x) || (X1>Map_v[1].x) ||
	   (Y1<Map_v[0].y) || (Y1>Map_v[3].y)) return;

  VLat=atan(sinh(M_PI * (2 * (Map_w[1].y-(yf*(Map_v[3].y-Y1))))))*(180.0 / M_PI);
  VLon=(Map_w[1].x-(xf*(Map_v[1].x-X1)))*360.0;

  MinRange=16.0;

  for(Data = (TADS_B_Aircraft *)ght_first(HashTable, &iterator,(const void **) &Key);
			  Data; Data = (TADS_B_Aircraft *)ght_next(HashTable, &iterator, (const void **)&Key))
	{
	  if (Data->HaveLatLon)
	  {
	   dlat= VLat-Data->Latitude;
	   dlon= VLon-Data->Longitude;
	   Range=sqrt(dlat*dlat+dlon*dlon);
	   if (Range<MinRange)
	   {
		Current_ICAO=Data->ICAO;
		MinRange=Range;
	   }
	  }
	}
	if (MinRange< 0.2)
	{
	  TADS_B_Aircraft * ADS_B_Aircraft =(TADS_B_Aircraft *)
			ght_get(HashTable,sizeof(Current_ICAO),
					&Current_ICAO);
	  if (ADS_B_Aircraft)
	  {
			if (!CPA_Hook)
			{
         AnsiString Text="Hooked Aircraft "+(AnsiString)GetAircraftDBInfo(ADS_B_Aircraft->ICAO) ;
         wchar_t *wtext= AnsiTowchar_t(Text);
			 if ((!TrackHook.Valid_CC) || (TrackHook.ICAO_CC != ADS_B_Aircraft->ICAO))
			   ClearTrajectoryState();
			 TrackHook.Valid_CC=true;
			 TrackHook.ICAO_CC=ADS_B_Aircraft->ICAO;
		 printf("%s\n\n",GetAircraftDBInfo(ADS_B_Aircraft->ICAO));
         Form1->SpVoice1->Speak(wtext, SpeechVoiceSpeakFlags::SVSFlagsAsync );  // Say Text and continue
         delete wtext;
         
         // Call Prediction API
         try{
           AnsiString Url="http://localhost:8001/predict/"+(AnsiString)ADS_B_Aircraft->HexAddr;
           PhaseLabel->Caption="Phase: Loading...";
           NetHTTPClientPrediction->Get(Url);
         }
         catch(...)
         {
             PhaseLabel->Caption="Phase: N/A";
         }
		}
		else
		{
		 TrackHook.Valid_CPA=true;
		 TrackHook.ICAO_CPA=ADS_B_Aircraft->ICAO;
        }
;
	  }

	}
	else
		{
			 if (!CPA_Hook)
			  {
			   TrackHook.Valid_CC=false;
	           ClearTrajectoryState();
	           ICAOLabel->Caption="N/A";
			   FlightNumLabel->Caption="N/A";
			   RouteLabel->Caption="N/A";
			   CLatLabel->Caption="N/A";
		   CLonLabel->Caption="N/A";
		   SpdLabel->Caption="N/A";
		   HdgLabel->Caption="N/A";
		   AltLabel->Caption="N/A";
		   MsgCntLabel->Caption="N/A";
         TrkLastUpdateTimeLabel->Caption="N/A";
           PhaseLabel->Caption="Phase: N/A";
           g_RiskOverlayCells.clear();
           g_HaveRiskOverlay=false;
		  }
		 else
		   {
			TrackHook.Valid_CPA=false;
			CpaTimeValue->Caption="None";
	        CpaDistanceValue->Caption="None";
           }
		}

 }
//---------------------------------------------------------------------------
void __fastcall TForm1::LatLon2XY(double lat,double lon, double &x, double &y)
{
 x=(Map_v[1].x-((Map_w[1].x-(lon/360.0))/xf));
 y= Map_v[3].y- (Map_w[1].y/yf)+ (asinh(tan(lat*M_PI/180.0))/(2*M_PI*yf));
}
//---------------------------------------------------------------------------
int __fastcall TForm1::XY2LatLon2(int x, int y,double &lat,double &lon )
{
  double Lat,Lon, dlat,dlon,Range;
  int X1,Y1;

  Y1=(ObjectDisplay->Height-1)-y;
  X1=x;

  if  ((X1<Map_v[0].x) || (X1>Map_v[1].x) ||
	   (Y1<Map_v[0].y) || (Y1>Map_v[3].y)) return -1;

  lat=atan(sinh(M_PI * (2 * (Map_w[1].y-(yf*(Map_v[3].y-Y1))))))*(180.0 / M_PI);
  lon=(Map_w[1].x-(xf*(Map_v[1].x-X1)))*360.0;

  return 0;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ZoomInClick(TObject *Sender)
{
  g_EarthView->SingleMovement(NAV_ZOOM_IN);
  ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::ZoomOutClick(TObject *Sender)
{
 g_EarthView->SingleMovement(NAV_ZOOM_OUT);

 ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Purge(void)
{
  uint32_t *Key;
  ght_iterator_t iterator;
  TADS_B_Aircraft* Data;
  void *p;
  __int64 CurrentTime=GetCurrentTimeInMsec();
  __int64  StaleTimeInMs=CSpinStaleTime->Value*1000;

  if (PurgeStale->Checked==false) return;

  for(Data = (TADS_B_Aircraft *)ght_first(HashTable, &iterator,(const void **) &Key);
			  Data; Data = (TADS_B_Aircraft *)ght_next(HashTable, &iterator, (const void **)&Key))
	{
	  if ((CurrentTime-Data->LastSeen)>=StaleTimeInMs)
	  {
	  p = ght_remove(HashTable,sizeof(*Key), Key);;
	  if (!p)
		ShowMessage("Removing the current iterated entry failed! This is a BUG\n");

	  delete Data;

	  }
	}
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Timer2Timer(TObject *Sender)
{
 Purge();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::PurgeButtonClick(TObject *Sender)
{
  uint32_t *Key;
  ght_iterator_t iterator;
  TADS_B_Aircraft* Data;
  void *p;

  for(Data = (TADS_B_Aircraft *)ght_first(HashTable, &iterator,(const void **) &Key);
			  Data; Data = (TADS_B_Aircraft *)ght_next(HashTable, &iterator, (const void **)&Key))
	{

	  p = ght_remove(HashTable,sizeof(*Key), Key);
	  if (!p)
		ShowMessage("Removing the current iterated entry failed! This is a BUG\n");

	  delete Data;

	}
}
//---------------------------------------------------------------------------
void __fastcall TForm1::InsertClick(TObject *Sender)
{
 Insert->Enabled=false;
 LoadARTCCBoundaries1->Enabled=false;
 Complete->Enabled=true;
 Cancel->Enabled=true;
 //Delete->Enabled=false;

 AreaTemp= new TArea;
 AreaTemp->NumPoints=0;
 AreaTemp->Name="";
 AreaTemp->Selected=false;
 AreaTemp->Triangles=NULL;

}
//---------------------------------------------------------------------------
void __fastcall TForm1::CancelClick(TObject *Sender)
{
 TArea *Temp;
 Temp= AreaTemp;
 AreaTemp=NULL;
 delete  Temp;
 Insert->Enabled=true;
 Complete->Enabled=false;
 Cancel->Enabled=false;
 LoadARTCCBoundaries1->Enabled=true;
 //if (Areas->Count>0)  Delete->Enabled=true;
 //else   Delete->Enabled=false;

}
//---------------------------------------------------------------------------
void __fastcall TForm1::CompleteClick(TObject *Sender)
{

  int or1=orientation2D_Polygon( AreaTemp->Points,AreaTemp->NumPoints);
  if (or1==0)
   {
	ShowMessage("Degenerate Polygon");
    CancelClick(NULL);
	return;
   }
  if (or1==CLOCKWISE)
  {
	DWORD i;

	memcpy(AreaTemp->PointsAdj,AreaTemp->Points,sizeof(AreaTemp->Points));
	for (i = 0; i <AreaTemp->NumPoints; i++)
	 {
	   memcpy(AreaTemp->Points[i],
			 AreaTemp->PointsAdj[AreaTemp->NumPoints-1-i],sizeof( pfVec3));
	 }
  }
  if (checkComplex( AreaTemp->Points,AreaTemp->NumPoints))
   {
	ShowMessage("Polygon is Complex");
	CancelClick(NULL);
	return;
   }

  AreaConfirm->ShowDialog();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::AreaListViewSelectItem(TObject *Sender, TListItem *Item,
      bool Selected)
{
   DWORD Count;
   TArea *AreaS=(TArea *)Item->Data;
   bool HaveSelected=false;
	Count=Areas->Count;
	for (unsigned int i = 0; i < Count; i++)
	 {
	   TArea *Area = (TArea *)Areas->Items[i];
	   if (Area==AreaS)
	   {
		if (Item->Selected)
		{
		 Area->Selected=true;
		 HaveSelected=true;
		}
		else
		 Area->Selected=false;
	   }
	   else
		 Area->Selected=false;

	 }
	if (HaveSelected)  Delete->Enabled=true;
	else Delete->Enabled=false;
	ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::DeleteClick(TObject *Sender)
{
 int i = 0;

 while (i < AreaListView->Items->Count)
  {
	if (AreaListView->Items->Item[i]->Selected)
	{
	 TArea *Area;
	 int Index;

	 Area=(TArea *)AreaListView->Items->Item[i]->Data;
	 for (Index = 0; Index < Areas->Count; Index++)
	 {
	  if (Area==Areas->Items[Index])
	  {
	   Areas->Delete(Index);
	   AreaListView->Items->Item[i]->Delete();
	   TTriangles *Tri=Area->Triangles;
	   while(Tri)
	   {
		TTriangles *temp=Tri;
		Tri=Tri->next;
		free(temp->indexList);
		free(temp);
	   }
	   delete Area;
	   break;
	  }
	 }
	}
	else
	{
	  ++i;
	}
  }
 //if (Areas->Count>0)  Delete->Enabled=true;
 //else   Delete->Enabled=false;

 ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::AreaListViewCustomDrawItem(TCustomListView *Sender,
	  TListItem *Item, TCustomDrawState State, bool &DefaultDraw)
{
   TRect   R;
   int Left;
  AreaListView->Canvas->Brush->Color = AreaListView->Color;
  AreaListView->Canvas->Font->Color = AreaListView->Font->Color;
  R=Item->DisplayRect(drBounds);
  AreaListView->Canvas->FillRect(R);

  AreaListView->Canvas->TextWidth(Item->Caption);

 AreaListView->Canvas->TextOut(2, R.Top, Item->Caption );

 Left = AreaListView->Column[0]->Width;

  for(int   i=0   ;i<Item->SubItems->Count;i++)
	 {
	  R=Item->DisplayRect(drBounds);
	  R.Left=R.Left+Left;
	   TArea *Area=(TArea *)Item->Data;
	  AreaListView->Canvas->Brush->Color=Area->Color;
	  AreaListView->Canvas->FillRect(R);
	 }

  if (Item->Selected)
	 {
	  R=Item->DisplayRect(drBounds);
	  AreaListView->Canvas->DrawFocusRect(R);
	 }
   DefaultDraw=false;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::DeleteAllAreas(void)
{
 int i = 0;

 while (AreaListView->Items->Count)
  {

	 TArea *Area;
	 int Index;

	 Area=(TArea *)AreaListView->Items->Item[i]->Data;
	 for (Index = 0; Index < Areas->Count; Index++)
	 {
	  if (Area==Areas->Items[Index])
	  {
	   Areas->Delete(Index);
	   AreaListView->Items->Item[i]->Delete();
	   TTriangles *Tri=Area->Triangles;
	   while(Tri)
	   {
		TTriangles *temp=Tri;
		Tri=Tri->next;
		free(temp->indexList);
		free(temp);
	   }
	   delete Area;
	   break;
	  }
	 }
  }

 ObjectDisplay->Repaint();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::FormMouseWheel(TObject *Sender, TShiftState Shift,
	  int WheelDelta, TPoint &MousePos, bool &Handled)
{
 if (WheelDelta>0)
	  g_EarthView->SingleMovement(NAV_ZOOM_IN);
 else g_EarthView->SingleMovement(NAV_ZOOM_OUT);
  ObjectDisplay->Repaint();
}                                  
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void __fastcall TTCPClientRawHandleThread::HandleInput(void)
{
  modeS_message mm;
  TDecodeStatus Status;

 // Form1->MsgLog->Lines->Add(StringMsgBuffer);
  if (Form1->RecordRawStream)
  {
   __int64 CurrentTime;
   CurrentTime=GetCurrentTimeInMsec();
   Form1->RecordRawStream->WriteLine(IntToStr(CurrentTime));
   Form1->RecordRawStream->WriteLine(StringMsgBuffer);
  }

  Status=decode_RAW_message(StringMsgBuffer, &mm);
  if (Status==HaveMsg)
  {
   TADS_B_Aircraft *ADS_B_Aircraft;
   uint32_t addr;

	addr = (mm.AA[0] << 16) | (mm.AA[1] << 8) | mm.AA[2];


	ADS_B_Aircraft =(TADS_B_Aircraft *) ght_get(Form1->HashTable,sizeof(addr),&addr);
	if (ADS_B_Aircraft)
	  {
      	//Form1->MsgLog->Lines->Add("Retrived");
      }
    else
	  {
  	   ADS_B_Aircraft= new TADS_B_Aircraft;
	   ADS_B_Aircraft->ICAO=addr;
	   snprintf(ADS_B_Aircraft->HexAddr,sizeof(ADS_B_Aircraft->HexAddr),"%06X",(int)addr);
	   ADS_B_Aircraft->NumMessagesSBS=0;
       ADS_B_Aircraft->NumMessagesRaw=0;
       ADS_B_Aircraft->VerticalRate=0;
	   ADS_B_Aircraft->HaveAltitude=false;
       ADS_B_Aircraft->HaveLatLon=false;
	   ADS_B_Aircraft->HaveSpeedAndHeading=false;
	   ADS_B_Aircraft->HaveFlightNum=false;
       ADS_B_Aircraft->HaveRoute=false;
       ADS_B_Aircraft->RequestedRoute=false;
       ADS_B_Aircraft->Route[0]=NULL;
	   ADS_B_Aircraft->SpriteImage=Form1->CurrentSpriteImage;
	   if (Form1->CycleImages->Checked)
		 Form1->CurrentSpriteImage=(Form1->CurrentSpriteImage+1)%Form1->NumSpriteImages;
	   if (ght_insert(Form1->HashTable,ADS_B_Aircraft,sizeof(addr), &addr) < 0)
		  {
			printf("ght_insert Error - Should Not Happen\n");
		  }
	  }

	  RawToAircraft(&mm,ADS_B_Aircraft);
  }
  else  printf("Raw Decode Error:%d\n",Status);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::RawConnectButtonClick(TObject *Sender)
{
 IdTCPClientRaw->Host=RawIpAddress->Text;
 IdTCPClientRaw->Port=30002;

 if ((RawConnectButton->Caption=="Raw Connect") && (Sender!=NULL))
 {
  try
   {
   IdTCPClientRaw->Connect();
   TCPClientRawHandleThread = new TTCPClientRawHandleThread(true);
   TCPClientRawHandleThread->UseFileInsteadOfNetwork=false;
   TCPClientRawHandleThread->FreeOnTerminate=TRUE;
   TCPClientRawHandleThread->Resume();
   }
   catch (const EIdException& e)
   {
    ShowMessage("Error while connecting: "+e.Message);
   }
 }
 else
  {
	TCPClientRawHandleThread->Terminate();
	IdTCPClientRaw->Disconnect();
	IdTCPClientRaw->IOHandler->InputBuffer->Clear();
	RawConnectButton->Caption="Raw Connect";
	RawPlaybackButton->Enabled=true;
  }
 }
//---------------------------------------------------------------------------
void __fastcall TForm1::IdTCPClientRawConnected(TObject *Sender)
{
   //SetKeepAliveValues(const AEnabled: Boolean; const ATimeMS, AInterval: Integer);
   IdTCPClientRaw->Socket->Binding->SetKeepAliveValues(true,60*1000,15*1000);
   RawConnectButton->Caption="Raw Disconnect";
   RawPlaybackButton->Enabled=false;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::IdTCPClientRawDisconnected(TObject *Sender)
{
  TCPClientRawHandleThread->Terminate();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::RawRecordButtonClick(TObject *Sender)
{
 if (RawRecordButton->Caption=="Raw Record")
 {
  if (RecordRawSaveDialog->Execute())
   {
	// First, check if the file exists.
	if (FileExists(RecordRawSaveDialog->FileName))
	  ShowMessage("File "+RecordRawSaveDialog->FileName+"already exists. Cannot overwrite.");
	else
	{
		// Open a file for writing. Creates the file if it doesn't exist, or overwrites it if it does.
	RecordRawStream= new TStreamWriter(RecordRawSaveDialog->FileName, false);
	if (RecordRawStream==NULL)
	  {
		ShowMessage("Cannot Open File "+RecordRawSaveDialog->FileName);
	  }
	 else RawRecordButton->Caption="Stop Raw Recording";
	}
  }
 }
 else
 {
   delete RecordRawStream;
   RecordRawStream=NULL;
   RawRecordButton->Caption="Raw Record";
 }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::RawPlaybackButtonClick(TObject *Sender)
{
  if ((RawPlaybackButton->Caption=="Raw Playback") && (Sender!=NULL))
 {
  if (PlaybackRawDialog->Execute())
   {
	// First, check if the file exists.
	if (!FileExists(PlaybackRawDialog->FileName))
	  ShowMessage("File "+PlaybackRawDialog->FileName+" does not exist");
	else
	{
		// Open a file for writing. Creates the file if it doesn't exist, or overwrites it if it does.
	PlayBackRawStream= new TStreamReader(PlaybackRawDialog->FileName);
	if (PlayBackRawStream==NULL)
	  {
		ShowMessage("Cannot Open File "+PlaybackRawDialog->FileName);
	  }
	 else {
		   TCPClientRawHandleThread = new TTCPClientRawHandleThread(true);
		   TCPClientRawHandleThread->UseFileInsteadOfNetwork=true;
		   TCPClientRawHandleThread->First=true;
		   TCPClientRawHandleThread->FreeOnTerminate=TRUE;
		   TCPClientRawHandleThread->Resume();
		   RawPlaybackButton->Caption="Stop Raw Playback";
           RawConnectButton->Enabled=false;
		  }
	}
  }
 }
 else
 {
   TCPClientRawHandleThread->Terminate();
   delete PlayBackRawStream;
   PlayBackRawStream=NULL;
   RawPlaybackButton->Caption="Raw Playback";
   RawConnectButton->Enabled=true;
 }
}
//---------------------------------------------------------------------------
// Constructor for the thread class
__fastcall TTCPClientRawHandleThread::TTCPClientRawHandleThread(bool value) : TThread(value)
{
	FreeOnTerminate = true; // Automatically free the thread object after execution
}
//---------------------------------------------------------------------------
// Destructor for the thread class
__fastcall TTCPClientRawHandleThread::~TTCPClientRawHandleThread()
{
	// Clean up resources if needed
}
//---------------------------------------------------------------------------
// Execute method where the thread's logic resides
void __fastcall TTCPClientRawHandleThread::Execute(void)
{
  __int64 Time,SleepTime;
  while (!Terminated)
  {
	if (!UseFileInsteadOfNetwork)
	 {
	  try {
		   if (!Form1->IdTCPClientRaw->Connected()) Terminate();
	       StringMsgBuffer=Form1->IdTCPClientRaw->IOHandler->ReadLn();
		  }
       catch (...)
		{
		 TThread::Synchronize(StopTCPClient);
		 break;
		}

	 }
	 else
	 {
	  try
        {
         if (Form1->PlayBackRawStream->EndOfStream)
           {
            printf("End Raw Playback 1\n");
            TThread::Synchronize(StopPlayback);
            break;
           }
		 StringMsgBuffer= Form1->PlayBackRawStream->ReadLine();
         Time=StrToInt64(StringMsgBuffer);
		 if (First)
	      {
		   First=false;
		   LastTime=Time;
		  }
		 SleepTime=Time-LastTime;
		 LastTime=Time;
		 if (SleepTime>0) Sleep(SleepTime);
         if (Form1->PlayBackRawStream->EndOfStream)
           {
            printf("End Raw Playback 2\n");
            TThread::Synchronize(StopPlayback);
            break;
           }
		 StringMsgBuffer= Form1->PlayBackRawStream->ReadLine();
		}
        catch (...)
		{
         printf("Raw Playback Exception\n");
		 TThread::Synchronize(StopPlayback);
		 break;
		}
	   }
     try
      {
	   // Synchronize method to safely access UI components
	   TThread::Synchronize(HandleInput);
      }
	 catch (...)
     {
      ShowMessage("TTCPClientRawHandleThread::Execute Exception 3");
	 }
  }
}
//---------------------------------------------------------------------------
void __fastcall TTCPClientRawHandleThread::StopPlayback(void)
{
 Form1->RawPlaybackButtonClick(NULL);
}
//---------------------------------------------------------------------------
void __fastcall TTCPClientRawHandleThread::StopTCPClient(void)
{
 Form1->RawConnectButtonClick(NULL);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::CycleImagesClick(TObject *Sender)
{
 CurrentSpriteImage=0;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::SBSConnectButtonClick(TObject *Sender)
{
 IdTCPClientSBS->Host=SBSIpAddress->Text;
 IdTCPClientSBS->Port=5002;

 if ((SBSConnectButton->Caption=="SBS Connect") && (Sender!=NULL))
 {
  try
   {
   IdTCPClientSBS->Connect();
   TCPClientSBSHandleThread = new TTCPClientSBSHandleThread(true);
   TCPClientSBSHandleThread->UseFileInsteadOfNetwork=false;
   TCPClientSBSHandleThread->FreeOnTerminate=TRUE;
   TCPClientSBSHandleThread->Resume();
   }
   catch (const EIdException& e)
   {
	ShowMessage("Error while connecting: "+e.Message);
   }
 }
 else
  {
	TCPClientSBSHandleThread->Terminate();
	IdTCPClientSBS->Disconnect();
    IdTCPClientSBS->IOHandler->InputBuffer->Clear();
	SBSConnectButton->Caption="SBS Connect";
	SBSPlaybackButton->Enabled=true;
  }

}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void __fastcall TTCPClientSBSHandleThread::HandleInput(void)
{
  modeS_message mm;
  TDecodeStatus Status;

 // Form1->MsgLog->Lines->Add(StringMsgBuffer);
  if (Form1->RecordSBSStream)
  {
   __int64 CurrentTime;
   CurrentTime=GetCurrentTimeInMsec();
   Form1->RecordSBSStream->WriteLine(IntToStr(CurrentTime));
   Form1->RecordSBSStream->WriteLine(StringMsgBuffer);
  }

  if (Form1->BigQueryCSV)
  {
    Form1->BigQueryCSV->WriteLine(StringMsgBuffer);
    Form1->BigQueryRowCount++;
	if (Form1->BigQueryRowCount>=BIG_QUERY_UPLOAD_COUNT)
	{
	 Form1->CloseBigQueryCSV();
	 //printf("string is:%s\n", Form1->BigQueryPythonScript.c_str());
	 RunPythonScript(Form1->BigQueryPythonScript,Form1->BigQueryPath+" "+Form1->BigQueryCSVFileName);
	 Form1->CreateBigQueryCSV();
	}
  }
  SBS_Message_Decode( StringMsgBuffer.c_str());

}
//---------------------------------------------------------------------------
// Constructor for the thread class
__fastcall TTCPClientSBSHandleThread::TTCPClientSBSHandleThread(bool value) : TThread(value)
{
	FreeOnTerminate = true; // Automatically free the thread object after execution
}
//---------------------------------------------------------------------------
// Destructor for the thread class
__fastcall TTCPClientSBSHandleThread::~TTCPClientSBSHandleThread()
{
	// Clean up resources if needed
}
//---------------------------------------------------------------------------
// Execute method where the thread's logic resides
void __fastcall TTCPClientSBSHandleThread::Execute(void)
{
  __int64 Time,SleepTime;
  while (!Terminated)
  {
	if (!UseFileInsteadOfNetwork)
	 {
	  try {
		   if (!Form1->IdTCPClientSBS->Connected()) Terminate();
	       StringMsgBuffer=Form1->IdTCPClientSBS->IOHandler->ReadLn();
		  }
       catch (...)
		{
		 TThread::Synchronize(StopTCPClient);
		 break;
		}

	 }
	 else
	 {
	  try
        {
         if (Form1->PlayBackSBSStream->EndOfStream)
           {
            printf("End SBS Playback 1\n");
            TThread::Synchronize(StopPlayback);
            break;
           }
		 StringMsgBuffer= Form1->PlayBackSBSStream->ReadLine();
         Time=StrToInt64(StringMsgBuffer);
		 if (First)
	      {
		   First=false;
		   LastTime=Time;
		  }
		 SleepTime=Time-LastTime;
		 LastTime=Time;
		 if (SleepTime>0) Sleep(SleepTime);
         if (Form1->PlayBackSBSStream->EndOfStream)
           {
            printf("End SBS Playback 2\n");
            TThread::Synchronize(StopPlayback);
            break;
           }
		 StringMsgBuffer= Form1->PlayBackSBSStream->ReadLine();
		}
        catch (...)
		{
         printf("SBS Playback Exception\n");
		 TThread::Synchronize(StopPlayback);
		 break;
		}
	   }
     try
      {
	   // Synchronize method to safely access UI components
	   TThread::Synchronize(HandleInput);
      }
	 catch (...)
     {
      ShowMessage("TTCPClientSBSHandleThread::Execute Exception 3");
	 }
  }
}
//---------------------------------------------------------------------------
void __fastcall TTCPClientSBSHandleThread::StopPlayback(void)
{
 Form1->SBSPlaybackButtonClick(NULL);
}
//---------------------------------------------------------------------------
void __fastcall TTCPClientSBSHandleThread::StopTCPClient(void)
{
 Form1->SBSConnectButtonClick(NULL);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::SBSRecordButtonClick(TObject *Sender)
{
 if (SBSRecordButton->Caption=="SBS Record")
 {
  if (RecordSBSSaveDialog->Execute())
   {
	// First, check if the file exists.
	if (FileExists(RecordSBSSaveDialog->FileName))
	  ShowMessage("File "+RecordSBSSaveDialog->FileName+"already exists. Cannot overwrite.");
	else
	{
		// Open a file for writing. Creates the file if it doesn't exist, or overwrites it if it does.
	RecordSBSStream= new TStreamWriter(RecordSBSSaveDialog->FileName, false);
	if (RecordSBSStream==NULL)
	  {
		ShowMessage("Cannot Open File "+RecordSBSSaveDialog->FileName);
	  }
	 else SBSRecordButton->Caption="Stop SBS Recording";
	}
  }
 }
 else
 {
   delete RecordSBSStream;
   RecordSBSStream=NULL;
   SBSRecordButton->Caption="SBS Record";
 }

}
//---------------------------------------------------------------------------
void __fastcall TForm1::SBSPlaybackButtonClick(TObject *Sender)
{
  if ((SBSPlaybackButton->Caption=="SBS Playback") && (Sender!=NULL))
 {
  if (PlaybackSBSDialog->Execute())
   {
	// First, check if the file exists.
	if (!FileExists(PlaybackSBSDialog->FileName))
	  ShowMessage("File "+PlaybackSBSDialog->FileName+" does not exist");
	else
	{
		// Open a file for writing. Creates the file if it doesn't exist, or overwrites it if it does.
	PlayBackSBSStream= new TStreamReader(PlaybackSBSDialog->FileName);
	if (PlayBackSBSStream==NULL)
	  {
		ShowMessage("Cannot Open File "+PlaybackSBSDialog->FileName);
	  }
	 else {
		   TCPClientSBSHandleThread = new TTCPClientSBSHandleThread(true);
		   TCPClientSBSHandleThread->UseFileInsteadOfNetwork=true;
		   TCPClientSBSHandleThread->First=true;
		   TCPClientSBSHandleThread->FreeOnTerminate=TRUE;
		   TCPClientSBSHandleThread->Resume();
		   SBSPlaybackButton->Caption="Stop SBS Playback";
           SBSConnectButton->Enabled=false;
		  }
	}
  }
 }
 else
 {
   TCPClientSBSHandleThread->Terminate();
   delete PlayBackSBSStream;
   PlayBackSBSStream=NULL;
   SBSPlaybackButton->Caption="SBS Playback";
   SBSConnectButton->Enabled=true;
 }

}
//---------------------------------------------------------------------------

void __fastcall TForm1::IdTCPClientSBSConnected(TObject *Sender)
{
   //SetKeepAliveValues(const AEnabled: Boolean; const ATimeMS, AInterval: Integer);
   IdTCPClientSBS->Socket->Binding->SetKeepAliveValues(true,60*1000,15*1000);
   SBSConnectButton->Caption="SBS Disconnect";
   SBSPlaybackButton->Enabled=false;
}
//---------------------------------------------------------------------------
void __fastcall TForm1::IdTCPClientSBSDisconnected(TObject *Sender)
{
  TCPClientSBSHandleThread->Terminate();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::TimeToGoTrackBarChange(TObject *Sender)
{
  _int64 hmsm;
  hmsm=TimeToGoTrackBar->Position*1000;
  TimeToGoText->Caption=TimeToChar(hmsm);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::LoadMap(int Type)
{
   AnsiString  HomeDir = ExtractFilePath(ExtractFileDir(Application->ExeName));
    if (Type==GoogleMaps)
   {
     HomeDir+= "..\\GoogleMap";
     if (LoadMapFromInternet) HomeDir+= "_Live\\";
     else  HomeDir+= "\\";
     std::string cachedir;
     cachedir=HomeDir.c_str();

     if (mkdir(cachedir.c_str()) != 0 && errno != EEXIST)
	    throw Sysutils::Exception("Can not create cache directory");

     g_Storage = new FilesystemStorage(cachedir,true);
     if (LoadMapFromInternet)
       {
	    g_Keyhole = new KeyholeConnection(GoogleMaps);
        g_Keyhole->SetSaveStorage(g_Storage);
	    g_Storage->SetNextLoadStorage(g_Keyhole);
	   }
    }
  else if (Type==SkyVector_VFR)
   {
     HomeDir+= "..\\VFR_Map";
     if (LoadMapFromInternet) HomeDir+= "_Live\\";
     else  HomeDir+= "\\";
     std::string cachedir;
     cachedir=HomeDir.c_str();

     if (mkdir(cachedir.c_str()) != 0 && errno != EEXIST)
	    throw Sysutils::Exception("Can not create cache directory");

     g_Storage = new FilesystemStorage(cachedir,true);
     if (LoadMapFromInternet)
       {
	    g_Keyhole = new KeyholeConnection(SkyVector_VFR);
        g_Keyhole->SetSaveStorage(g_Storage);
	    g_Storage->SetNextLoadStorage(g_Keyhole);
	   }
    }
  else if (Type==SkyVector_IFR_Low)
   {
     HomeDir+= "..\\IFR_Low_Map";
     if (LoadMapFromInternet) HomeDir+= "_Live\\";
     else  HomeDir+= "\\";
     std::string cachedir;
     cachedir=HomeDir.c_str();

     if (mkdir(cachedir.c_str()) != 0 && errno != EEXIST)
	    throw Sysutils::Exception("Can not create cache directory");

     g_Storage = new FilesystemStorage(cachedir,true);
     if (LoadMapFromInternet)
       {
	    g_Keyhole = new KeyholeConnection(SkyVector_IFR_Low);
        g_Keyhole->SetSaveStorage(g_Storage);
	    g_Storage->SetNextLoadStorage(g_Keyhole);
	   }
    }
  else if (Type==SkyVector_IFR_High)
   {
     HomeDir+= "..\\IFR_High_Map";
     if (LoadMapFromInternet) HomeDir+= "_Live\\";
     else  HomeDir+= "\\";
     std::string cachedir;
     cachedir=HomeDir.c_str();

     if (mkdir(cachedir.c_str()) != 0 && errno != EEXIST)
	    throw Sysutils::Exception("Can not create cache directory");

     g_Storage = new FilesystemStorage(cachedir,true);
     if (LoadMapFromInternet)
       {
	    g_Keyhole = new KeyholeConnection(SkyVector_IFR_High);
        g_Keyhole->SetSaveStorage(g_Storage);
	    g_Storage->SetNextLoadStorage(g_Keyhole);
	   }
    }
   g_GETileManager = new TileManager(g_Storage);
   g_MasterLayer = new GoogleLayer(g_GETileManager);

   g_EarthView = new FlatEarthView(g_MasterLayer);
   g_EarthView->Resize(ObjectDisplay->Width,ObjectDisplay->Height);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::MapComboBoxChange(TObject *Sender)
{
  double    m_Eyeh= g_EarthView->m_Eye.h;
  double    m_Eyex= g_EarthView->m_Eye.x;
  double    m_Eyey= g_EarthView->m_Eye.y;


  Timer1->Enabled=false;
  Timer2->Enabled=false;
  delete g_EarthView;
  if (g_GETileManager) delete g_GETileManager;
  delete g_MasterLayer;
  delete g_Storage;
  if (LoadMapFromInternet)
  {
   if (g_Keyhole) delete g_Keyhole;
  }
  if (MapComboBox->ItemIndex==0)   LoadMap(GoogleMaps);

  else if (MapComboBox->ItemIndex==1)  LoadMap(SkyVector_VFR);

  else if (MapComboBox->ItemIndex==2)  LoadMap(SkyVector_IFR_Low);

  else if (MapComboBox->ItemIndex==3)   LoadMap(SkyVector_IFR_High);

   g_EarthView->m_Eye.h =m_Eyeh;
   g_EarthView->m_Eye.x=m_Eyex;
   g_EarthView->m_Eye.y=m_Eyey;
   Timer1->Enabled=true;
   Timer2->Enabled=true;

}
//---------------------------------------------------------------------------

void __fastcall TForm1::BigQueryCheckBoxClick(TObject *Sender)
{
 if (BigQueryCheckBox->State==cbChecked) CreateBigQueryCSV();
 else {
	   CloseBigQueryCSV();
	   RunPythonScript(BigQueryPythonScript,BigQueryPath+" "+BigQueryCSVFileName);
	  }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::CreateBigQueryCSV(void)
{
    AnsiString  HomeDir = ExtractFilePath(ExtractFileDir(Application->ExeName));
    BigQueryCSVFileName="BigQuery"+UIntToStr(BigQueryFileCount)+".csv";
    BigQueryRowCount=0;
    BigQueryFileCount++;
    BigQueryCSV=new TStreamWriter(HomeDir+"..\\BigQuery\\"+BigQueryCSVFileName, false);
    if (BigQueryCSV==NULL)
	  {
		ShowMessage("Cannot Open BigQuery CSV File "+HomeDir+"..\\BigQuery\\"+BigQueryCSVFileName);
        BigQueryCheckBox->State=cbUnchecked;
	  }
	AnsiString Header=AnsiString("Message Type,Transmission Type,SessionID,AircraftID,HexIdent,FlightID,Date_MSG_Generated,Time_MSG_Generated,Date_MSG_Logged,Time_MSG_Logged,Callsign,Altitude,GroundSpeed,Track,Latitude,Longitude,VerticalRate,Squawk,Alert,Emergency,SPI,IsOnGround");
	BigQueryCSV->WriteLine(Header);
}
//--------------------------------------------------------------------------
void __fastcall TForm1::CloseBigQueryCSV(void)
{
    if (BigQueryCSV)
    {
     delete BigQueryCSV;
     BigQueryCSV=NULL;
    }
}
//--------------------------------------------------------------------------
	 static void RunPythonScript(AnsiString scriptPath,AnsiString args)
     {
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;

        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        ZeroMemory(&pi, sizeof(pi));

        AnsiString commandLine = "python " + scriptPath+" "+args;
        char* cmdLineCharArray = new char[strlen(commandLine.c_str()) + 1];
		strcpy(cmdLineCharArray, commandLine.c_str());
	#define  LOG_PYTHON 1
	#if LOG_PYTHON
        //printf("%s\n", cmdLineCharArray);
        SECURITY_ATTRIBUTES sa;
        sa.nLength = sizeof(sa);
	    sa.lpSecurityDescriptor = NULL;
        sa.bInheritHandle = TRUE;
		HANDLE h = CreateFileA(Form1->BigQueryLogFileName.c_str(),
		FILE_APPEND_DATA,
        FILE_SHARE_WRITE | FILE_SHARE_READ,
        &sa,
		OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
		NULL );

        si.hStdInput = NULL;
	    si.hStdOutput = h;
	    si.hStdError = h; // Redirect standard error as well, if needed
	    si.dwFlags |= STARTF_USESTDHANDLES;
    #endif
        if (!CreateProcessA(
            nullptr,          // No module name (use command line)
            cmdLineCharArray, // Command line
            nullptr,          // Process handle not inheritable
            nullptr,          // Thread handle not inheritable
	 #if LOG_PYTHON
            TRUE,
     #else
            FALSE,            // Set handle inheritance to FALSE
     #endif
            CREATE_NO_WINDOW, // Don't create a console window
			nullptr,          // Use parent's environment block
            nullptr,          // Use parent's starting directory
            &si,             // Pointer to STARTUPINFO structure
            &pi))             // Pointer to PROCESS_INFORMATION structure
         {
            std::cerr << "CreateProcess failed (" << GetLastError() << ").\n";
            delete[] cmdLineCharArray;
            return;
         }

        // Optionally, detach from the process
        CloseHandle(pi.hProcess);
		CloseHandle(pi.hThread);
		delete[] cmdLineCharArray;
    }

 //--------------------------------------------------------------------------
void __fastcall TForm1::UseSBSRemoteClick(TObject *Sender)
{
 SBSIpAddress->Text="data.adsbhub.org";
}
//---------------------------------------------------------------------------

void __fastcall TForm1::UseSBSLocalClick(TObject *Sender)
{
 SBSIpAddress->Text="128.237.96.41";
}
//---------------------------------------------------------------------------
static bool DeleteFilesWithExtension(AnsiString dirPath, AnsiString extension)
 {
	AnsiString searchPattern = dirPath + "\\*." + extension;
	WIN32_FIND_DATAA findData;

	HANDLE hFind = FindFirstFileA(searchPattern.c_str(), &findData);
    #define INVALID_HANDLE_VALUE_XX ((HANDLE)(::LONG_PTR)-1)
	if (hFind == INVALID_HANDLE_VALUE_XX) {
		return false; // No files found or error
	}

	do {
		AnsiString filePath = dirPath + "\\" + findData.cFileName;
		if (DeleteFileA(filePath.c_str()) == 0) {
			FindClose(hFind);
			return false; // Failed to delete a file
		}
	} while (FindNextFileA(hFind, &findData) != 0);

	FindClose(hFind);
	return true;
}
static bool IsFirstRow=true;
static bool CallBackInit=false;
//---------------------------------------------------------------------------
 static int CSV_callback_ARTCCBoundaries (struct CSV_context *ctx, const char *value)
{
  int    rc = 1;
  static char LastArea[512];
  static char Area[512];
  static char Lat[512];
  static char Lon[512];
  int    Deg,Min,Sec,Hsec;
  char   Dir;

   if (ctx->field_num==0)
   {
	strcpy(Area,value);
   }
   else if (ctx->field_num==3)
   {
	strcpy(Lat,value);
   }
   else if (ctx->field_num==4)
   {
    strcpy(Lon,value);
   }

   if (ctx->field_num == (ctx->num_fields - 1))
   {

	float fLat, fLon;
   if (!IsFirstRow)
   {
	 if (!CallBackInit)
	 {
	  strcpy(LastArea,Area);
	  CallBackInit=true;
	 }
	   if(strcmp(LastArea,Area)!=0)
		{

		 if (FinshARTCCBoundary())
		   {
			printf("Load ERROR ID %s\n",LastArea);
		   }
		 else printf("Loaded ID %s\n",LastArea);
		 strcpy(LastArea,Area);
		 }
	   if (Form1->AreaTemp==NULL)
		   {
			Form1->AreaTemp= new TArea;
			Form1->AreaTemp->NumPoints=0;
			Form1->AreaTemp->Name=Area;
			Form1->AreaTemp->Selected=false;
			Form1->AreaTemp->Triangles=NULL;
			 printf("Loading ID %s\n",Area);
		   }
	   if (sscanf(Lat,"%2d%2d%2d%2d%c",&Deg,&Min,&Sec,&Hsec,&Dir)!=5)
		 printf("Latitude Parse Error\n");
	   fLat=Deg+Min/60.0+Sec/3600.0+Hsec/360000.00;
	   if (Dir=='S') fLat=-fLat;

	   if (sscanf(Lon,"%3d%2d%2d%2d%c",&Deg,&Min,&Sec,&Hsec,&Dir)!=5)
		 printf("Longitude Parse Error\n");
	   fLon=Deg+Min/60.0+Sec/3600.0+Hsec/360000.00;
	   if (Dir=='W') fLon=-fLon;
	   //printf("%f, %f\n",fLat,fLon);
	   if (Form1->AreaTemp->NumPoints<MAX_AREA_POINTS)
	   {
		Form1->AreaTemp->Points[Form1->AreaTemp->NumPoints][1]=fLat;
		Form1->AreaTemp->Points[Form1->AreaTemp->NumPoints][0]=fLon;
		Form1->AreaTemp->Points[Form1->AreaTemp->NumPoints][2]=0.0;
		Form1->AreaTemp->NumPoints++;
	   }
		else printf("Max Area Points Reached\n");

   }
   if (IsFirstRow) IsFirstRow=false;
   }
  return(rc);
}
//---------------------------------------------------------------------------
bool __fastcall TForm1::LoadARTCCBoundaries(AnsiString FileName)
{
  CSV_context  csv_ctx;
   memset (&csv_ctx, 0, sizeof(csv_ctx));
   csv_ctx.file_name = FileName.c_str();
   csv_ctx.delimiter = ',';
   csv_ctx.callback  = CSV_callback_ARTCCBoundaries;
   csv_ctx.line_size = 2000;
   IsFirstRow=true;
   CallBackInit=false;
   if (!CSV_open_and_parse_file(&csv_ctx))
    {
	  printf("Parsing of \"%s\" failed: %s\n", FileName.c_str(), strerror(errno));
      return (false);
	}
   if ((Form1->AreaTemp!=NULL) && (Form1->AreaTemp->NumPoints>0))
   {
     char Area[512];
     strcpy(Area,Form1->AreaTemp->Name.c_str());
     if (FinshARTCCBoundary())
	    {
        printf("Loaded ERROR ID %s\n",Area);
	    }
        else printf("Loaded ID %s\n",Area);
   }
   printf("Done\n");
   return(true);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::LoadARTCCBoundaries1Click(TObject *Sender)
{
   LoadARTCCBoundaries(ARTCCBoundaryDataPathFileName);
}
//---------------------------------------------------------------------------
static int FinshARTCCBoundary(void)
{
  int or1=orientation2D_Polygon( Form1->AreaTemp->Points,Form1->AreaTemp->NumPoints);
  if (or1==0)
   {
	TArea *Temp;
	Temp= Form1->AreaTemp;
	Form1->AreaTemp=NULL;
	delete  Temp;
	printf("Degenerate Polygon\n");
	return(-1);
   }
  if (or1==CLOCKWISE)
  {
	DWORD i;

	memcpy(Form1->AreaTemp->PointsAdj,Form1->AreaTemp->Points,sizeof(Form1->AreaTemp->Points));
	for (i = 0; i <Form1->AreaTemp->NumPoints; i++)
	 {
	   memcpy(Form1->AreaTemp->Points[i],
			 Form1->AreaTemp->PointsAdj[Form1->AreaTemp->NumPoints-1-i],sizeof( pfVec3));
	 }
  }
  if (checkComplex( Form1->AreaTemp->Points,Form1->AreaTemp->NumPoints))
   {
	TArea *Temp;
	Temp= Form1->AreaTemp;
	Form1->AreaTemp=NULL;
	delete  Temp;
	printf("Polygon is Complex\n");
    return(-2);
   }
  DWORD Row,Count,i;


 Count=Form1->Areas->Count;
 for (i = 0; i < Count; i++)
 {
  TArea *Area = (TArea *)Form1->Areas->Items[i];
  if (Area->Name==Form1->AreaTemp->Name) {

   TArea *Temp;
   Temp= Form1->AreaTemp;
   printf("Duplicate Area Name %s\n",Form1->AreaTemp->Name.c_str());;
   Form1->AreaTemp=NULL;
   delete  Temp;
   return(-3);
   }
 }

 triangulatePoly(Form1->AreaTemp->Points,Form1->AreaTemp->NumPoints,
				 &Form1->AreaTemp->Triangles);

 Form1->AreaTemp->Color=TColor(PopularColors[CurrentColor]);
 CurrentColor++ ;
 CurrentColor=CurrentColor%NumColors;
 Form1->Areas->Add(Form1->AreaTemp);
 Form1->AreaListView->Items->BeginUpdate();
 Form1->AreaListView->Items->Add();
 Row=Form1->AreaListView->Items->Count-1;
 Form1->AreaListView->Items->Item[Row]->Caption=Form1->AreaTemp->Name;
 Form1->AreaListView->Items->Item[Row]->Data=Form1->AreaTemp;
 Form1->AreaListView->Items->Item[Row]->SubItems->Add("");
 Form1->AreaListView->Items->EndUpdate();
 Form1->AreaTemp=NULL;
 return 0 ;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::SpSharedRecoContext1Recognition(TObject *Sender, long StreamNumber,
          Variant StreamPosition, SpeechRecognitionType RecognitionType,
          ISpeechRecoResult *Result)
{
 Memo1->Lines->Add( Result->PhraseInfo->GetText(0, -1, True) );
}
//---------------------------------------------------------------------------

void __fastcall TForm1::LIstenClick(TObject *Sender)
{
    Memo1->Visible=true;
	SpSharedRecoContext1->EventInterests = SpeechRecoEvents::SREAllEvents;
	SRGrammar=SpSharedRecoContext1->CreateGrammar(Variant(0));
	SRGrammar->CmdSetRuleIdState(0, SpeechRuleState::SGDSActive);
	SRGrammar->DictationSetState(SpeechRuleState::SGDSActive);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::NetHTTPClientPredictionRequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse)
{
    if (AResponse && AResponse->StatusCode == 200)
    {
        AnsiString JsonStr = AResponse->ContentAsString(TEncoding::ASCII);
        
        // Simple string parsing to extract phase
        // JSON format: {"icao":"...", "phase":"CLIMB", ...}
        char *ptr = stristr(JsonStr.c_str(), "\"phase\":");
        if (ptr)
        {
            ptr += 8; // Skip "phase":
            while (*ptr == ' ' || *ptr == '"') ptr++; // Skip spaces and quote
            
            char phase[64];
            int i = 0;
            while (*ptr != '"' && *ptr != ',' && *ptr != '}' && i < 63)
            {
                phase[i++] = *ptr++;
            }
            phase[i] = '\0';
            
            PhaseLabel->Caption = "Phase: " + (AnsiString)phase;
        }
        else
        {
            PhaseLabel->Caption = "Phase: N/A";
        }
    }
    else
    {
        PhaseLabel->Caption = "Phase: N/A";
    }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::NetHTTPClientPredictionRequestError(TObject *Sender, const UnicodeString AError)
{
    PhaseLabel->Caption = "Phase: N/A";
}
//---------------------------------------------------------------------------
void __fastcall TForm1::NetHTTPClientWeatherRequestCompleted(TObject *Sender, _di_IHTTPResponse AResponse)
{
    // Weather response is handled synchronously in SendPredictedRoutePointsToBackend.
}
//---------------------------------------------------------------------------
void __fastcall TForm1::NetHTTPClientWeatherRequestError(TObject *Sender, const UnicodeString AError)
{
    // Non-fatal: weather overlay should not affect phase UI.
}
//---------------------------------------------------------------------------
