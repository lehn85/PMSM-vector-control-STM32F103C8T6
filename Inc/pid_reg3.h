/* ==================================================================================
File name:       PID_REG3.H  (IQ version)                    
=====================================================================================*/

#ifndef __PIDREG3_H__
#define __PIDREG3_H__

typedef struct {  _iq  Ref;   			// Input: Reference input 
				  _iq  Fdb;   			// Input: Feedback input 
				  _iq  Err;				// Variable: Error
				  _iq  Kp;				// Parameter: Proportional gain
				  _iq  Up;				// Variable: Proportional output 
				  _iq  Ui;				// Variable: Integral output 
				  _iq  Ud;				// Variable: Derivative output 	
				  _iq  OutPreSat; 		// Variable: Pre-saturated output
				  _iq  OutMax;		    // Parameter: Maximum output 
				  _iq  OutMin;	    	// Parameter: Minimum output
				  _iq  Out;   			// Output: PID output 
				  _iq  SatErr;			// Variable: Saturated difference
				  _iq  Ki;			    // Parameter: Integral gain
				  _iq  Kc;		     	// Parameter: Integral correction gain
				  _iq  Kd; 		        // Parameter: Derivative gain
				  _iq  Up1;		   	    // History: Previous proportional output
		 	 	} PIDREG3;	            

typedef PIDREG3 *PIDREG3_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG3_DEFAULTS { 0, 			\
                           0, 			\
                           0, 			\
                           _IQ(1.3), 	\
                           0, 			\
                           0, 			\
                           0, 			\
                           0, 			\
                           _IQ(1), 		\
                           _IQ(-1), 	\
                           0, 			\
                           0, 			\
                           _IQ(0.02), 	\
                           _IQ(0.5), 	\
                           _IQ(1.05), 	\
                           0, 			\
              			  }

/*------------------------------------------------------------------------------
 	PID Macro Definition
------------------------------------------------------------------------------*/


#define PID_MACRO(v)																					\
	v.Err = v.Ref - v.Fdb; 									/* Compute the error */						\
	v.Up= _IQmpy(v.Kp,v.Err);								/* Compute the proportional output */		\
	v.Ui= v.Ui + _IQmpy(v.Ki,v.Up) + _IQmpy(v.Kc,v.SatErr);	/* Compute the integral output */			\
	v.OutPreSat= v.Up + v.Ui;								/* Compute the pre-saturated output */		\
	v.Out = _IQsat(v.OutPreSat, v.OutMax, v.OutMin);		/* Saturate the output */					\
	v.SatErr = v.Out - v.OutPreSat;							/* Compute the saturate difference */		\
	v.Up1 = v.Up;											/* Update the previous proportional output */

#endif // __PIDREG3_H__

// Add the lines below if derivative output is needed following the integral update
// v.Ud = _IQmpy(v.Kd,(v.Up - v.Up1)); 
// v.OutPreSat = v.Up + v.Ui + v.Ud; 
