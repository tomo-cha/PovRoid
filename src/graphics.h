#define NUMPIXELS 50
#define Div 60

uint32_t pic [Div][NUMPIXELS] = {
	{0x000000, 0x000000, 0x000000, 0x120B05, 0x302D2B, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x0F0F07, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x231F1B, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x1A150F, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x1A150F, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x120C06, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x1A150F, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0B0704, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x322F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120C05, 0x120C05, 0x120C05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x130D06, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120C05, 0x120C05, 0x120C05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x2E2B29, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x0B0703, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x231F1B, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x090603, 0x120B05, 0x2D2A27, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x020100, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x201B16, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x040301, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x1C1711, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120C05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x1C1711, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x27231F, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x221D18, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x2E2B29, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x322F2D, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0A0704, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x030200, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x030200, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312E2C, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x140E07, 0x140E07, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120D09, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x2B2724, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x24201C, 0x120B05, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x302D2B, 0x120B05, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x231F1B, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x0F0F07, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x1A150F, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120C06, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x1A150F, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x1A150F, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120C05, 0x120C05, 0x120C05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120C05, 0x120C05, 0x120C05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x0B0704, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x322F2D, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x130D06, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x0B0703, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x2D2A27, 0x120B05, 0x090603, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x201B16, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x2E2B29, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x231F1B, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x040301, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x312F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x020100, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120C05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x27231F, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x1C1711, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x1C1711, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x322F2D, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x221D18, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x2E2B29, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x312E2C, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x030200, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x0A0704, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120D09, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x030200, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x000000, 0x120B05, 0x120B05, 0x140E07, 0x140E07, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x120B05, 0x120B05, 0x32302E, 0x32302E, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x000000, 0x120B05, 0x32302E, 0x2B2724, 0x120B05, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
	{0x000000, 0x000000, 0x000000, 0x120B05, 0x24201C, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x32302E, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x120B05, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000},
};