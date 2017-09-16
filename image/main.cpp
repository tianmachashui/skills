#include "ImageController.h"

int main(int argc, char* argv[])//"/home/yue/Downloads/image/image/a.png"
{
		
	ImageController * l_obj=NULL;
	l_obj = new ImageController(argv[1]);
	l_obj->run();
	return OK;
}
