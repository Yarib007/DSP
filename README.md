# DSP

# Digital Signal processing

The signal processing unit consist of a generic digital filter structure, it is implemented in the class filter and extended in the class composed filter. The filter and composed filter classes mainly employ polymorphism, encapsulation OOP characteristics, as well as composite and factory design patterns. It can be instantiated as many filters and composes filters as needed. The composed filter is capable to drive as many filters as desired.

# Discrete filter (DSP)

This class implements a generic discrete filter. This implementation executes a linear system mechanism with feedback and feedforward coefficients in double precision. The mechanism implemented follows the same structure as any straightforward filter code. The filter class is defined in filter.h.
In the next code is exhibited the filter class definitions and interfaces.
```C
typedef struct
{
    const unsigned char order;  // Filter order
    const double *      a_coef; // Feedback filter coefficients
    const double *      b_coef; // Feedforward filter coefficients
} FilterParameters;

typedef struct Filter_public Filter;

struct Filter_public
{
    void   (* reset)  (Filter * obj);
    double (* process)(Filter * obj, double u);
    void   (* delete) (Filter ** obj);
    double (* output) (Filter * obj);
    const FilterParameters * (* access_parameters) (Filter * obj);
};

Filter * Filter_new(const FilterParameters * parameters);
```
The encapsulation of data and function members is made by type casting. First, it is allocated memory needed for the bases private class, then it is fulfilled with proper data types (data and function pointers as virtual table); secondly, it is out-casted to a base type class reducing its data member accessibility.
The following code shows the private implementation.

```C
typedef struct // class
{	// public:
	void   (* reset)  (Filter * obj);
	double (* process)(Filter * obj, double u);
	void   (* delete) (Filter ** obj);
	double (* output) (Filter * obj);
	const FilterParameters * (* access_parameters) (Filter * obj);
	// private:
	const FilterParameters * parameters;
	double  * states;
	double    y;
} Filter_private;

static void Filter_init_virtual_table(Filter_private * filter)
{
    if (filter != NULL)
    {
        filter->reset = Filter_reset;
        filter->process = Filter_process;
        filter->output = Filter_output;
        filter->access_parameters = Filter_access_parameters;
        filter->delete = Filter_delete;
    }
}

Filter * Filter_new(const FilterParameters * parameters)
{
    Filter_private * filter = NULL;

    if (   parameters != NULL
        && parameters->a_coef != NULL
        && parameters->b_coef != NULL
        && parameters->order > 0)
    {
        filter = (Filter_private *) malloc(sizeof(Filter_private));

        if (filter != NULL)
        {
            memset(filter, 0x00, sizeof(Filter_private));

            Filter_init_virtual_table(filter);

            filter->parameters = parameters;

            Filter_reset((Filter *) filter);
        }
    }

    return (Filter *) filter;
}

static void   Filter_delete (Filter ** obj)
{
    if (obj != NULL && *obj != NULL)
    {
        Filter_private * filter = (Filter_private *) *obj;

        if (filter->states != NULL)
        {
            free(filter->states);
        }

        free(filter);

        *obj = NULL;
    }
}
```
The filter process is implemented in the following function.
```C
static double Filter_process(Filter * obj, double u)
{
    double y = (double)0.0;
    if (obj != NULL)
    {
        Filter_private * filter = (Filter_private *) obj;

        if (filter->parameters != NULL)
        {
            const double * A = filter->parameters->a_coef;
            const double * B = filter->parameters->b_coef;
            const unsigned char n = filter->parameters->order;

            double * x = filter->states;

            if ((n > 0) && (A != NULL) && (B != NULL) && (x != NULL))
            {
                unsigned char i;

                y = B[0] * u + x[0];
                
                for (i = 1; i < n; i ++) { x[i-1] = B[i] * u - A[i] * y + x[i]; }
                
                x[n-1] = B[n] * u - A[n] * y;

                filter->y = y;
            }
        }
    }
    return y;
}
```
The next code present a factory creation of a filter instance, it is created by using its constructor and destroyed by using its destructor. The filter coefficients have been taken from MATLAB.

```C
#define HIGH_PASS_CUTOFF_FREC_HZ  0.83 
#define HIGH_PASS_FILTER_ORDER    4 

#define F0_A0  ((const double)0000000000000001)
#define F0_A1  ((const double)-3.9863177122115889e+00)
#define F0_A2  ((const double)5.9590466614474735e+00)
#define F0_A3  ((const double)-3.9591398122141528e+00)
#define F0_A4  ((const double)9.8641086372476394e-01)

#define F0_B0  ((const double)9.9318219059987367e-01)
#define F0_B1  ((const double)-3.9727287623994947e+00)
#define F0_B2  ((const double)5.9590931435992420e+00)
#define F0_B3  ((const double)-3.9727287623994947e+00)
#define F0_B4  ((const double)9.9318219059987367e-01)

const double A_high_pass [] = {F0_A0, F0_A1, F0_A2, F0_A3, F0_A4};
const double B_high_pass [] = {F0_B0, F0_B1, F0_B2, F0_B3, F0_B4};

const FilterParameters high_pass_filter_parameters = {HIGH_PASS_FILTER_ORDER, A_high_pass, B_high_pass};

Filter * filter = Filter_new(&high_pass_filter_parameters);
```

The next line of code shows the usage of the filter to process a signal.
```C
void AnyClass::timerInterruptHandler(void)
{
primitiveSignal = Poxi_ADC->read_analog()

filteredSignal = filter->process(filter, primitiveSignal);

stdIO::fprintf(file, "%.4f  %.4f\n", primitiveSignal, filteredSignal);
}
```

This line of code is used to delete a filter instance.
```C
filter->delete(&filter);
```

For more details about the code implementation it can be referred the actual source code.


# Composed discrete filter

The composed filter is intended to be an extension of the base filter class and to provide more features. The composed filter basically connects and wraps any number of filter instances, and takes statistics when processing signals. The filter statistics, after the signal processing is one of the most useful feature of this class, and this can be reconfigurable on flight. The composed filter class is defined in composedfilter.h.
Class definition.
```C
#include "filter.h"

typedef struct
{
    double magnitude;
    double time;
} DiscretePoint;

typedef struct
{
    DiscretePoint maxOutput;
    DiscretePoint minOutput;
    DiscretePoint maxInput;
    DiscretePoint minInput;
    DiscretePoint currentOutput;
    DiscretePoint lastInput;
    DiscretePoint lastZeroCross;
    DiscretePoint secLastZeroCross;
    double        fundamentalFrec;
} FilterStatistics;

typedef enum
{
    STATISTICS_OFF   = 0x00,
    MAX_OUTPUT       = 0x01,
    MIN_OUTPUT       = 0x02,
    MAX_INPUT        = 0x04,
    MIN_INPUT        = 0x08,
    FUNDAMENTAL_FREC = 0x10
} FilterStatisticsFlags;

typedef struct ComposedFilter_public ComposedFilter;

struct ComposedFilter_public // class
{
    void   (* reset)  (ComposedFilter * obj);
    double (* process)(ComposedFilter * obj, double u);
    void   (* delete)         (ComposedFilter ** obj);
    double (* processSignal)  (ComposedFilter * obj,
                               double * input, double * output, unsigned int length);
    void   (* setSampleTime)  (ComposedFilter * obj, double time);
    void   (* setupStatistics)(ComposedFilter * obj, int flags);
    void   (* resetStatistics)(ComposedFilter * obj);
    FilterStatistics (* getStatistics)(ComposedFilter * obj);
};

ComposedFilter * ComposedFilter_new (int number_of_filters, ...);
```

The constructor of the class is a variable parameter function, it receives a variable number of filter instances, and the first parameter is the number of filters that will build up the composed filter.
Here is an example of two filters given to a composed filter, low and high pass filter, giving as a result a band pass filter.
``` C
ComposedFilter * bandpassFilter;
Filter *         highpassfilter;
Filter *         lowpassfilter;

highpassfilter = Filter_new(&high_pass_filter_parameters);
lowpassfilter = Filter_new(&low_pass_filter_parameters);

bandpassFilter = ComposedFilter_new(2, highpassfilter, lowpassfilter);

Here is listed the code to process signal with a composed filter.
void AnyClass_timerInterruptHandler(void)
{
primitiveSignal = Poxi_ADC->read_analog()

filteredSignal = bandpassFilter->process(bandpassFilter, primitiveSignal);

fprintf(file,"%.10f  %.10f\n", primitiveSignal, filteredSignal);
}
```

Deletion of a composed filter.
```C
bandpassFilter->delete(&bandpassFilter);
```

The composed filter is able to obtain statistics when processing a signal.
Setup of statistics flags. To get proper statistics it is needed to set the sample time used for the design of the discreet filters. The statistics are setup by the usage of the statistics flags.

# Filter design and simulation
This DSP is designed, calculated and simulated using MATLAB. The Butterworth filter is the filter type selected for filter coefficients calculation. MATLAB scripts are written to perform automated filter calculations and simulations.
The Butterworth filter calculation and its step and frequency responses are coded in the script file POXI_INIT.M.

```MATLAB
clc
disp('Poxi initialization ...')
clear
format long
%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%%%%%
fh = 3;     % Low pass cut-off frecuency
Lowpass_Order = 6;
fl = 50/60;    % High pass cut-off frecuency
Highpass_Order = 4;
T = .001;    % Sample time
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate DSP.H for firmware implementation
 Butterworth_POXI(fh, Lowpass_Order, fl, Highpass_Order, T)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = 1/T;
 
[BH,AH]=butter(Highpass_Order,fl/(F/2),'high');
[BL,AL]=butter(Lowpass_Order,fh/(F/2),'low');
 
%%%%%%%%%%%%%%%%%%%% Transfer functions %%%%%%%%%%%%%%%%%%%%
DHighPass = filt(BH,AH,T)
DLowPass = filt(BL,AL,T)
 
CHighPass = d2c(DHighPass);
CLowPass = d2c(DLowPass);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[BHC,AHC] = tfdata(CHighPass,'v');
[BLC,ALC] = tfdata(CLowPass,'v');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frecuency responce
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% High Pass filter
w = linspace(0, 2*fl, 100)';
magH = freqresp(DHighPass, w, 'Hz');
magH = abs(magH(:));
 
figure
subplot(6,2,1);
plot(w,magH);
grid
title('POXI - High pass filter frequency response')
xlabel('Hz');
ylabel('Mag');
subplot(6,2,2);
step(DHighPass);
% Low Pass filter
w = linspace(0, 10*fh, 100)';
magL = freqresp(DLowPass, w, 'Hz');
magL = abs(magL(:));
subplot(6,2,3);
plot(w,magL);
grid
title('POXI - Low pass filter frequency response')
xlabel('Hz');
ylabel('Mag');
subplot(6,2,4);
step(DLowPass);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load ..\JAVA_APP\poxi\poxi_adc.dat
subplot(6,2,[5,6]);
time = poxi_adc(:,1);
u    = poxi_adc(:,2);
plot(time,u)
title('POXI ADC SIGNAL')
ylabel('Mag');
legend('ADC Original')
grid
 
subplot(6,2,[7,8]);
plot(time,filter(BH,AH,filter(BL,AL,u)));
title('LOW-HIGH PASS FILTERED SIGNAL')
ylabel('Mag');
legend('Low High Pass Filtered')
grid
 
subplot(6,2,[9,10]);
plot(time,filter(BL,AL,u));
title('LOW PASS FILTERED SIGNAL')
ylabel('Mag');
legend('Low pass Filtered')
grid
 
subplot(6,2,[11,12]);
plot(time,filter(BH,AH,u));
title('HIGH PASS FILTERED SIGNAL')
xlabel('Time (S)');
ylabel('Mag');
legend('High pass Filtered')
grid
 
%signal.time = (T:T:length(u)*T)';
signal.time = time;
signal.signals.values = [u];
signal.signals.dimensions = 1;
 
disp('Done !')
```

Results:
```
Poxi initialization ...
Poxi filter design ...
Generating "..\poxi.sdk\poxi\src\dsp\DSP.H" ...
Done !

DHighPass =
 
  0.9932 - 3.973 z^-1 + 5.959 z^-2 - 3.973 z^-3 + 0.9932 z^-4
  -----------------------------------------------------------
    1 - 3.986 z^-1 + 5.959 z^-2 - 3.959 z^-3 + 0.9864 z^-4
 
Sample time: 0.001 seconds
Discrete-time transfer function.


DLowPass =
 
  6.759e-13 + 4.055e-12 z^-1 + 1.014e-11 z^-2 + 1.352e-11 z^-3 + 1.014e-11 z^-4 + 4.055e-12 z^-5 + 6.759e-13 z^-6
  ---------------------------------------------------------------------------------------------------------------
                 1 - 5.927 z^-1 + 14.64 z^-2 - 19.28 z^-3 + 14.29 z^-4 - 5.646 z^-5 + 0.9298 z^-6
 
Sample time: 0.001 seconds
Discrete-time transfer function.

Done !
```
Best regards,

-Yarib Nevárez
