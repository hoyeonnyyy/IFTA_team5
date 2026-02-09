//----------------------------------------------------------------------------

#ifndef AirportLookupH
#define AirportLookupH

#include <vcl.h>

bool InitAirportLookup(AnsiString fileName);
bool LookupAirportByCode(const AnsiString &code, double &lat, double &lon);
bool AirportLookupLoaded();

#endif
