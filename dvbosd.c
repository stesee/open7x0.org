/*
 * dvbosd.c: Implementation of the DVB On Screen Display
 *
 * See the main source file 'vdr.c' for copyright information and
 * how to reach the author.
 *
 * $Id: dvbosd.c 1.30 2006/01/28 14:24:04 kls Exp $
 */

#include "dvbosd.h"
//M7X0 BEGIN AK
// Nearly the complete File is changed.
// OSD is done via Linux Framebuffer-Device
// At the moment runs 720x576@32-Bit Interlaced

 
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/unistd.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include "tools.h"
#include <errno.h>

// --- cDvbOsd ---------------------------------------------------------------

class cDvbOsd : public cOsd {
private:
	uint32_t *fbMem;
	int fbWidth;
	int fbHeight;
	int fbBpp;
	bool fbInterlaced;
	bool shown;

public:
	cDvbOsd(int Left, int Top, uint32_t *FbMem ,
			int FbWidth, int FbHeight, int FbBpp,bool FbInterlaced);
	virtual ~cDvbOsd();
	virtual void Flush(void);
  };

cDvbOsd::cDvbOsd(int Left, int Top, uint32_t *FbMem,int FbWidth, 
	int FbHeight,int FbBpp, bool FbInterlaced)
:cOsd(Left, Top)
{
	shown = false;
	
	fbMem = FbMem;
	fbWidth=FbWidth;
	fbHeight=FbHeight;
	fbBpp=FbBpp;
	fbInterlaced=FbInterlaced;
	
}

cDvbOsd::~cDvbOsd()
{
	if (shown) {
		for (int i=0, i2= (fbHeight>>1) * fbWidth;
				i2 < fbWidth*fbHeight ; i+=fbWidth, i2+=fbWidth){
			memset(fbMem+i,0,fbWidth*(fbBpp>>3));
			memset(fbMem+i2,0,fbWidth*(fbBpp>>3));
		}
		msync(fbMem,fbWidth*fbHeight*(fbBpp>>3),MS_ASYNC);
	}
}

void cDvbOsd::Flush(void)
{
	cBitmap *Bitmap;
	int x1, y1, y1Field1, y1Field2; 
	// Used for the Dirty-Rectangle in the Bitmap.
	int x1BitmapDirty, x2BitmapDirty, y1BitmapDirty, y2BitmapDirty;
	int i,j,k;
	int NumColors;
	int bitmapWidth;
	const tColor *Colors;
	const tIndex *bitmapData;
	
	shown = true;
	
	for (i=0; (Bitmap = GetBitmap(i)) != NULL; i++) {
		if(!Bitmap->Dirty(x1BitmapDirty, y1BitmapDirty, 
				x2BitmapDirty , y2BitmapDirty))
			continue;
		
		// Clear Dirty-Area
		Bitmap->Clean();
		
		Colors = Bitmap->Colors(NumColors);
		bitmapData = Bitmap->Data(0,0);
		
		if(!Colors || !bitmapData) 
			continue;
		
		bitmapWidth= Bitmap->Width();
		
		x1 = Bitmap->X0() + Left(); 
		y1 = y1BitmapDirty + Bitmap->Y0() + Top();
		
		if((y1&1)==0) {
			// Bitmaps First Dirty Line is in the first Field
			y1Field1= (y1 >> 1)*fbWidth;
			y1Field2= ((fbHeight>>1)+(y1>>1))*fbWidth;
		} else {
			// Bitmaps First Dirty Line is in the second Field
			y1Field1= ((fbHeight>>1)+(y1>>1))*fbWidth;
			y1Field2= ((y1 >> 1)+1)*fbWidth;
		}
		
		y1Field1+=x1;
		y1Field2+=x1;
		
    	for (j = y1BitmapDirty*bitmapWidth ;
				y1BitmapDirty < y2BitmapDirty; 
				y1BitmapDirty+=2, j += bitmapWidth<<1,
				y1Field1 += fbWidth , y1Field2 += fbWidth) {
			// Maybe this loop is faster if splited up in two (for each field)
			// This can happen due to caching of the CPU
			for (k=x1BitmapDirty ; k <= x2BitmapDirty; k++){
				fbMem[y1Field1+k]=Colors[bitmapData[j+k]];
				fbMem[y1Field2+k]=Colors[bitmapData[j+k+bitmapWidth]];
			}
		} 
		
		if ( y1BitmapDirty == y2BitmapDirty ){
			// Copy last Dirty-Line of Bitmap
			for (k=x1BitmapDirty ; k <= x2BitmapDirty; k++){
				fbMem[y1Field1+k]=Colors[bitmapData[j+k]];
			}
		}
		
		
	}

	msync(fbMem,fbWidth*fbHeight*(fbBpp>>3),MS_ASYNC);   
	
}

// --- cDvbOsdProvider -------------------------------------------------------

cDvbOsdProvider::cDvbOsdProvider(int OsdDev)
{
	osdDev = OsdDev;
	fbMem=NULL;
	
	if (osdDev < 0) {
		esyslog("ERROR: illegal OSD device handle (%d)!", osdDev);
		return;
	}
	
	struct fb_var_screeninfo scrInf;
	if (ioctl(osdDev,FBIOGET_VSCREENINFO,&scrInf)<0) {
		esyslog("ERROR: cannot get Framebuffer-Settings !");
		return;
	}
	 	
	fbWidth=scrInf.xres;
	fbHeight=scrInf.yres;
	fbBpp=scrInf.bits_per_pixel;
	fbInterlaced=scrInf.vmode&FB_VMODE_INTERLACED;
			
	fbMem= (uint32_t*) mmap(0, fbWidth * fbHeight * (fbBpp>>3),
			PROT_READ|PROT_WRITE, MAP_SHARED, osdDev, 0);
					
	if (fbMem==((uint32_t*)-1)) {
		esyslog("ERROR: cannot mmap Framebuffer (%d)!",errno); 
		fbMem=NULL;
	}
	
	if ( !fbMem || fbWidth!=720 || fbHeight!=576 || 
			fbBpp!=32 || !fbInterlaced )
		esyslog("ERROR: illegal or unsupported Framebuffer: %u x %u x %u !",
				fbWidth,fbHeight,fbBpp);
	else {
		memset(fbMem,0,fbWidth*fbHeight*(fbBpp>>3));    
		msync(fbMem,fbWidth*fbHeight*(fbBpp>>3),MS_ASYNC);
	}
	
}

cDvbOsdProvider::~cDvbOsdProvider(){
	if(fbMem){
		munmap(fbMem,fbWidth*fbHeight*(fbBpp>>3));
	}
}

cOsd *cDvbOsdProvider::CreateOsd(int Left, int Top)
{
	if ( fbMem && fbWidth==720 && fbHeight==576 && 
			fbBpp==32 && fbInterlaced ) // For Safty
		return new cDvbOsd(Left, Top, fbMem, fbWidth, fbHeight, 
				fbBpp, fbInterlaced);
	else // Argh, something went uglly wrong, return a dummy osd.
		return new cOsd(Left,Top);
}

//M7X0 END AK
