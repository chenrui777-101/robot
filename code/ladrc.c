#include "zf_common_headfile.h"

TD TD_LR_Angle;//×óÓÒ¸ú×ÙÎ¢·ÖÆ÷
TD TD_FB_Angle;//Ç°ºó¸ú×ÙÎ¢·ÖÆ÷

ESO ESO_LR_Angle;//×óÓÒÀ©ÕÅ×´Ì¬¹Û²âÆ÷
ESO ESO_FB_Angle;//Ç°ºóÀ©ÕÅ×´Ì¬¹Û²âÆ÷

/************************************************************/
//          v0:Ä¿±êÖµ£¨ÆÚÍûexpect£©
//          TD¸ú×ÙÎ¢·ÖÆ÷
/************************************************************/
void TD_Update(TD *td, float v0)
{
    float fh= -td->r*td->r*(td->v1-v0)-2*td->r*td->v2;
    td->v1+=td->v2*td->h;
    td->v2+=fh*td->h;
}

/**************************************************************/
//          eso->u:pid¼ÆËãµÄ¿ØÖÆÁ¿    measured_value:¹Û²âÁ¿£¨Êµ¼ÊÖµActual£©
//          ESOÀ©ÕÅ×´Ì¬¹Û²âÆ÷
/**************************************************************/
void ESO_Update(ESO *eso, float measured_value)
{
    float Beita_01=3*eso->w0;
    float Beita_02=3*eso->w0*eso->w0;
    float Beita_03=eso->w0*eso->w0*eso->w0;

    float e= eso->z1-measured_value;
    eso->z1+= (eso->z2 - Beita_01*e)*eso->h;
    eso->z2+= (eso->z3 - Beita_02*e + eso->b0*eso->u)*eso->h;
    eso->z3+=-Beita_03*e*eso->h;
}


// ¸¨Öúº¯Êı£¬ÓÃÓÚ¼ÆËã·ûºÅ
int sign(float x)
{
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}
