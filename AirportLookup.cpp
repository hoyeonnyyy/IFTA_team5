//----------------------------------------------------------------------------

#pragma hdrstop

#include <ctype.h>
#include <fstream>
#include <map>
#include <stdlib.h>
#include <string>
#include <vector>

#include "AirportLookup.h"

//----------------------------------------------------------------------------
#pragma package(smart_init)

typedef struct
{
  double Lat;
  double Lon;
} TAirportCoord;

static std::map<std::string, TAirportCoord> AirportByIata;
static std::map<std::string, TAirportCoord> AirportByIcao;
static bool                                 AirportDataLoaded = false;

//----------------------------------------------------------------------------
static std::string TrimStd(const std::string &value)
{
  size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return "";

  size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

//----------------------------------------------------------------------------
static std::string NormalizeCode(const std::string &value)
{
  std::string out = TrimStd(value);
  for (size_t i = 0; i < out.size(); i++)
    out[i] = (char)toupper((unsigned char)out[i]);

  if (out.empty() || out == "\\N")
    out.clear();

  return out;
}

//----------------------------------------------------------------------------
static bool ParseAirportLookupLine(const std::string &line,
                                   std::string &type,
                                   std::string &code,
                                   double &lat,
                                   double &lon)
{
  std::vector<std::string> fields;
  std::string token;
  size_t start = 0;

  while (true)
  {
    size_t comma = line.find(',', start);
    if (comma == std::string::npos)
    {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }

  if (fields.size() < 4)
    return false;

  type = NormalizeCode(fields[0]);
  code = NormalizeCode(fields[1]);

  char *endLat = NULL;
  char *endLon = NULL;
  lat = strtod(fields[2].c_str(), &endLat);
  lon = strtod(fields[3].c_str(), &endLon);

  if ((endLat == fields[2].c_str()) || (endLon == fields[3].c_str()))
    return false;

  return !(type.empty() || code.empty());
}

//----------------------------------------------------------------------------
bool InitAirportLookup(AnsiString fileName)
{
  AirportByIata.clear();
  AirportByIcao.clear();
  AirportDataLoaded = false;

  std::ifstream in(fileName.c_str());
  if (!in.is_open())
    return false;

  std::string line;
  bool first = true;

  while (std::getline(in, line))
  {
    if (line.empty())
      continue;

    if (first)
    {
      first = false;
      if (line.find("code_type") == 0)
        continue;
    }

    std::string type;
    std::string code;
    double lat = 0.0;
    double lon = 0.0;

    if (!ParseAirportLookupLine(line, type, code, lat, lon))
      continue;

    TAirportCoord coord;
    coord.Lat = lat;
    coord.Lon = lon;

    if (type == "IATA")
      AirportByIata[code] = coord;
    else if (type == "ICAO")
      AirportByIcao[code] = coord;
  }

  AirportDataLoaded = (!AirportByIata.empty() || !AirportByIcao.empty());
  return AirportDataLoaded;
}

//----------------------------------------------------------------------------
bool LookupAirportByCode(const AnsiString &code, double &lat, double &lon)
{
  std::string key = NormalizeCode(code.c_str());
  if (key.empty())
    return false;

  std::map<std::string, TAirportCoord>::const_iterator itIata = AirportByIata.find(key);
  if (itIata != AirportByIata.end())
  {
    lat = itIata->second.Lat;
    lon = itIata->second.Lon;
    return true;
  }

  std::map<std::string, TAirportCoord>::const_iterator itIcao = AirportByIcao.find(key);
  if (itIcao != AirportByIcao.end())
  {
    lat = itIcao->second.Lat;
    lon = itIcao->second.Lon;
    return true;
  }

  if (key.size() == 4)
  {
    std::string iataGuess = key.substr(1);
    itIata = AirportByIata.find(iataGuess);
    if (itIata != AirportByIata.end())
    {
      lat = itIata->second.Lat;
      lon = itIata->second.Lon;
      return true;
    }
  }

  return false;
}

//----------------------------------------------------------------------------
bool AirportLookupLoaded()
{
  return AirportDataLoaded;
}
