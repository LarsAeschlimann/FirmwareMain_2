#include "shiftregister.h"


char button_get_active_state(char encoder)
{
	if(encoder == HWADDR_BUTTON_CONTRAST)		return 0;
	if(encoder == HWADDR_BUTTON_SHADOW)			return 0;
	if(encoder == HWADDR_BUTTON_BLACK)			return 0;
	if(encoder == HWADDR_BUTTON_WHITE)		  return 0;
	if(encoder == HWADDR_BUTTON_LIGHTS)			return 0;
	if(encoder == HWADDR_BUTTON_EXPOSURE)		return 0;
	if(encoder == HWADDR_BUTTON_CLARITY)		return 0;
	if(encoder == HWADDR_BUTTON_DYNAMIC)		return 0;
	if(encoder == HWADDR_BUTTON_ENCSAT)			return 0;
							                             
	if(encoder == HWADDR_BUTTON_RED)				return 0;
	if(encoder == HWADDR_BUTTON_ORANGE)			return 0;
	if(encoder == HWADDR_BUTTON_YELLOW)			return 0;
	if(encoder == HWADDR_BUTTON_GREEN)			return 0;
	if(encoder == HWADDR_BUTTON_AQUA)				return 0;
	if(encoder == HWADDR_BUTTON_BLUE)				return 0;
	if(encoder == HWADDR_BUTTON_PURPLE)			return 0;
	if(encoder == HWADDR_BUTTON_MAGENTA)		return 0;
							                             
	if(encoder == HWADDR_BUTTON_PROG)				return 0;
	if(encoder == HWADDR_BUTTON_CROP)				return 0;
	                                   
	if(encoder == HWADDR_BUTTON_UNDO)				return 1;
	if(encoder == HWADDR_BUTTON_REDO)				return 1;
	if(encoder == HWADDR_BUTTON_FULL)				return 1;
	if(encoder == HWADDR_BUTTON_COLOR_BW)		return 1;
	if(encoder == HWADDR_BUTTON_SEL_HUE)		return 1;
	if(encoder == HWADDR_BUTTON_SEL_SAT)		return 1;
	if(encoder == HWADDR_BUTTON_SEL_LUM)		return 1;
	if(encoder == HWADDR_BUTTON_STAR_1)			return 1;
	if(encoder == HWADDR_BUTTON_STAR_2)			return 1;
	if(encoder == HWADDR_BUTTON_STAR_3)			return 1;
	if(encoder == HWADDR_BUTTON_STAR_4)			return 1;
	if(encoder == HWADDR_BUTTON_STAR_5)			return 1;
	if(encoder == HWADDR_BUTTON_COPY)				return 1;
	if(encoder == HWADDR_BUTTON_PASTE)			return 1;
	if(encoder == HWADDR_BUTTON_FN)					return 1;
	if(encoder == HWADDR_BUTTON_PICK)				return 1;
	if(encoder == HWADDR_BUTTON_ZOOM)				return 1;
	if(encoder == HWADDR_BUTTON_RIGHT)			return 1;
	if(encoder == HWADDR_BUTTON_LEFT)				return 1;
	if(encoder == HWADDR_BUTTON_UP)					return 1;
	if(encoder == HWADDR_BUTTON_DOWN)				return 1;
	if(encoder == HWADDR_BUTTON_BEFOREAFTER)return 1;
	if(encoder == HWADDR_BUTTON_DEVELOP)		return 1;
	
	return 1;
}
