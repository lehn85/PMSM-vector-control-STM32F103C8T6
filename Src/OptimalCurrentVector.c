/*
 * OptimalCurrentVector.c
 *
 *  Created on: 8 Apr, 2017
 *      Author: PhuongLe
 */

#include <stdio.h>
#include "IQmathLib.h"
#include "OptimalCurrentVector.h"

int16_t tblId[81] =
		{
				//speed= 0	0.125	0.25	0.375	0.5		0.625	0.75	0.875	1
				0, 0, 0, 0, 0, -3446, -7318, -10083, -12157, //torque=0
				0, 0, 0, 0, 0, -3499, -7381, -10156, -12241,
				0, 0, 0, 0, 0, -3657, -7571, -10379, -12496,
				0, 0, 0, 0, 0, -3923, -7892, -10757, -12932,
				0, 0, 0, 0, 0, -4301, -8351, -11302, -13083,
				0, 0, 0, 0, 0, -4796, -8960, -11874, -13083,
				0, 0, 0, 0, 0, -5417, -9732, -11874, -13083,
				0, 0, 0, 0, 0, -6175, -10010, -11874, -13083,
				0, 0, 0, 0, -460, -6920, -10010, -11874, -13083,
		};

int16_t tblIq[81] =
		{
				//speed=0	0.125	0.25	0.375	0.5		0.625	0.75	0.875	1
				0, 0, 0, 0, 0, 0, 0, 0, 0,
				1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560, 1560,
				3120, 3120, 3120, 3120, 3120, 3120, 3120, 3120, 3120,
				4681, 4681, 4681, 4681, 4681, 4681, 4681, 4681, 4681,
				6241, 6241, 6241, 6241, 6241, 6241, 6241, 6241, 5102,
				7801, 7801, 7801, 7801, 7801, 7801, 7801, 7498, 5102,
				9362, 9362, 9362, 9362, 9362, 9362, 9362, 7498, 5102,
				10922, 10922, 10922, 10922, 10922, 10922, 9848, 7498, 5102,
				12483, 12483, 12483, 12483, 12483, 12219, 9848, 7498, 5102,

		};

void init_optimalVectorCurrent(LookupTableIdIq *tbl)
{
	//tbl size is (Tc+1) x (Sc+1) (row x column)
	tbl->Tc = 8;
	tbl->Sc = 8;
	tbl->tblId = tblId;
	tbl->tblIq = tblIq;
}

void calc_optimalCurrentVector(LookupTableIdIq *tbl, _iq s, _iq t, _iq* id,
		_iq* iq)
{
	uint16_t signT = t >= 0;
	_iq a11, a12, a21, a22;
	_iq id11, id12, id21 = 0, id22;
	_iq iq11, iq12, iq21, iq22;
	_iq ds, dt;
	int16_t pos = 0;
	int16_t iS = 0, iT = 0;

	t = t > 0 ? t : -t;	//absolute
	s = s > 0 ? s : -s;	//absolute
//	if (s>_IQ(1))
//		s = _IQ(1);
//	if (t>_IQ(1))
//		t = _IQ(1);

	// calc column
	iS = s * tbl->Sc >> 24;
	//if (iS<0) iS = 0;
	if (iS > tbl->Sc - 1)
		iS = tbl->Sc - 1;
	ds = s * tbl->Sc - _IQ(iS);

	// calc max torque possible with input speed s
	// this may not be required if tblId, tblIq use value at max curve for the points outside of max curve
//	_iq tmax = _IQmpy(_IQ15toIQ(tbl->tblMaxTorque[iS]),_IQ(1)-ds)+_IQmpy(_IQ15toIQ(tbl->tblMaxTorque[iS+1]),ds);
//	if (t>tmax)
//		t=tmax;

	// calc row to interpolate
	iT = t * tbl->Tc >> 24;
	//if (iT<0) iT=0;
	if (iT > tbl->Tc - 1)
		iT = tbl->Tc - 1;
	dt = t * tbl->Tc - _IQ(iT);

	// calc coefficient
	a11 = _IQmpy(_IQ(1) - ds, _IQ(1) - dt);
	a12 = _IQmpy(ds, _IQ(1) - dt);
	a21 = _IQmpy(_IQ(1) - ds, dt);
	a22 = _IQmpy(ds, dt);

	// pos in table id,iq
	pos = iT * (tbl->Sc + 1) + iS;

	id11 = _IQ15toIQ(tbl->tblId[pos]);
	id12 = _IQ15toIQ(tbl->tblId[pos + 1]);
	id21 = _IQ15toIQ(tbl->tblId[pos + tbl->Sc + 1]);
	id22 = _IQ15toIQ(tbl->tblId[pos + tbl->Sc + 2]);

	*id = _IQmpy(id11,a11) + _IQmpy(id12, a12) + _IQmpy(id21, a21) + _IQmpy(id22, a22);

	iq11 = _IQ15toIQ(tbl->tblIq[pos]);
	iq12 = _IQ15toIQ(tbl->tblIq[pos + 1]);
	iq21 = _IQ15toIQ(tbl->tblIq[pos + tbl->Sc + 1]);
	iq22 = _IQ15toIQ(tbl->tblIq[pos + tbl->Sc + 2]);

	*iq = _IQmpy(iq11,a11) + _IQmpy(iq12, a12) + _IQmpy(iq21, a21) + _IQmpy(iq22, a22);
	if (!signT)
		*iq = -(*iq);
}
