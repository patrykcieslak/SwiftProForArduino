#include "uarm_coord_convert.h"

//               /\          |          /              |            \
//       ARM_A  /  \  ARM_B  |    ARM_A/               |             \ ARM_B
//           __/    \        |        /_____  AngleA   |   angleB ____\
//          | center         |                         |
//     origin

/*
	ARMA angle -> anglea
	ARMB angle -> angleb
	BASE angle -> anglec
*/
bool is_angle_legal(float anglea, float angleb, float anglec)
{

	if (isnan(anglea) || isnan(angleb) || isnan(anglec))
	{

		return false;
	}
	if ((anglea > ARMA_MAX_ANGLE) || (anglea < ARMA_MIN_ANGLE))
	{

		return false;
	}
	if ((angleb > ARMB_MAX_ANGLE) || (angleb < ARMB_MIN_ANGLE))
	{
		return false;
	}
	if ((180 - anglea - angleb > ARMA_ARMB_MAX_ANGLE) || (180 - anglea - angleb < ARMA_ARMB_MIN_ANGLE))
	{
		return false;
	}
	if ((anglec > BASE_MAX_ANGLE) || (anglec < BASE_MIN_ANGLE))
	{
		return false;
	}

	return true;
}

void get_current_angle(float x_step, float y_step, float z_step, float *anglea, float *angleb, float *anglec)
{
	*anglea = (uarm.arml_angle + x_step / steps_per_angle);
	*angleb = (uarm.armr_angle + y_step / steps_per_angle);
	*anglec = (uarm.base_angle - 90 + z_step / steps_per_angle);
}

void angle_to_coord(float anglea, float angleb, float anglec, float *x, float *y, float *z)
{
	float radiana = anglea / RAD_TO_DEG; // <! angle to radin
	float radianb = angleb / RAD_TO_DEG;
	float radianc = anglec / RAD_TO_DEG;

	float _stretch = cos(radiana) * ARM_A + cos(radianb) * ARM_B;
	float _height = sin(radiana) * ARM_A - sin(radianb) * ARM_B;

	*x = (_stretch + length_center_to_origin) * cos(radianc);
	*y = (_stretch + length_center_to_origin) * sin(radianc);
	*z = _height + Z_BASIC;
}

void coord_to_angle(float x, float y, float z, float *anglea, float *angleb, float *anglec)
{

	double _stretch = sqrt(x * x + y * y) - length_center_to_origin; //_stretch = sqrt(x^2+y^2)
	double _height = z - Z_BASIC;

	// get angle A
	double xx = _stretch * _stretch + _height * _height;
	double xxx = ARM_A2 - ARM_B2 + xx;

	*anglea = acos((_stretch * xxx - _height * sqrt(4.0 * ARM_A2 * xx - xxx * xxx)) / (xx * 2.0 * ARM_A)) * RAD_TO_DEG;
	// get angle B
	xxx = ARM_B2 - ARM_A2 + xx;
	*angleb = acos((_stretch * xxx + _height * sqrt(4.0 * ARM_B2 * xx - xxx * xxx)) / (xx * 2.0 * ARM_B)) * RAD_TO_DEG;
	if (_height > (ARM_A * sin(*anglea / RAD_TO_DEG)))
	{
		*angleb = -*angleb;
	}

	if (fabs(_height) > (ARM_B * sin(*angleb / RAD_TO_DEG)) && _height < 0.0)
	{
		*anglea = -*anglea;
	}

	// get the base rotation angle
	if (x == 0)
	{
		if (y > 0)
		{
			*anglec = 90;
		}
		else
		{
			*anglec = -90;
		}
	}
	else if (x < 0)
	{
		*anglec = NAN; // <! cann't arrive
	}
	else
	{
		*anglec = atan(y / x) * RAD_TO_DEG;
	}
}

void get_current_step(float anglea, float angleb, float anglec, float *x_step, float *y_step, float *z_step)
{
	*x_step = (int)((anglea - uarm.arml_angle) * steps_per_angle);
	*y_step = (int)((angleb - uarm.armr_angle) * steps_per_angle);
	*z_step = (int)((anglec - uarm.base_angle + 90) * steps_per_angle);
}

void step_to_coord(float x_step, float y_step, float z_step, float *x, float *y, float *z)
{
	float anglea, angleb, anglec;
	get_current_angle(x_step, y_step, z_step, &anglea, &angleb, &anglec);
	angle_to_coord(anglea, angleb, anglec, x, y, z);
}

void coord_to_step(float x, float y, float z, float *x_step, float *y_step, float *z_step)
{
	float anglea, angleb, anglec;
	coord_to_angle(x, y, z, &anglea, &angleb, &anglec);
	get_current_step(anglea, angleb, anglec, x_step, y_step, z_step);
}

void coord_arm2effect(float *x, float *y, float *z)
{
	float anglec;

	if (*x == 0)
	{
		if (*y > 0)
		{
			anglec = 90 / RAD_TO_DEG;
		}
		else
		{
			anglec = -90 / RAD_TO_DEG;
		}
	}
	else if (*x < 0)
	{
		anglec = NAN; // <! cann't arrive
	}
	else
	{
		anglec = atan(*y / *x);
	}

	*x += uarm.param.front_offset * cos(anglec);
	*y += uarm.param.front_offset * sin(anglec);
	*z -= uarm.param.high_offset;
}

void coord_effect2arm(float *x, float *y, float *z)
{
	float anglec;

	if (*x == 0)
	{
		if (*y > 0)
		{
			anglec = 90 / RAD_TO_DEG;
		}
		else
		{
			anglec = -90 / RAD_TO_DEG;
		}
	}
	else if (*x < 0)
	{
		anglec = NAN; // <! cann't arrive
	}
	else
	{
		anglec = atan(*y / *x);
	}

	*x -= uarm.param.front_offset * cos(anglec);
	*y -= uarm.param.front_offset * sin(anglec);
	*z += uarm.param.high_offset;
}
