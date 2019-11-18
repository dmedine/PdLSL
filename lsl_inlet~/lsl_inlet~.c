/**************** lsl_inlet ****************/
/* Written by David Medine on behalf of    */
/* Brain Products                          */ 
/* 15/5/2017                               */
/* Released under the GPL                  */
/* This software is free and open source   */
/*******************************************/


#include "m_pd.h"
#include "lsl_c.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


#ifdef _WIN32
#include "windows.h"
#define TID  HANDLE
#define MUTEX CRITICAL_SECTION
#else
#include <unistd.h>
#include "pthread.h"
#endif

//pd boilerplate:
static t_class *lsl_inlet_tilde_class;

typedef struct _lsl_inlet_tilde{

  t_object    x_obj;

  // pd outlets
  t_outlet   **sig_outlets;         // pd multiplexed signal outlet
  int        nout;                  // number of channels
  t_outlet   *ts_outlet;            // timestamp outlet
  t_sample   **lcl_outs;            // for convenience in the processing loop

  t_sample   **sig_buffs;           // ring buffers for holding lsl chunks as they arrive
  t_sample   *ts_buf;               // ring buffer for timestamps
  int        buflen;                // length of ring buffer
  int        nchannels;             // number of channels in the lsl_inlet
  int        longbuflen;            // nchannels * buflen (for sig_buf)
  int        widx;                  // write index for ring buffer
  int        ridx;                  // read index for ring buffer
  double     m_dReadIdx;            // interpolated read point;
  float      sampling_per;          // time to add to first timestamp to get the chunk stamp
  int        connected;             // flag to say if we are connected or not
  int        ready;                 // flag to say if we have enough samples to start reading
  int        lag;                   // lag time to buffer up

  // do simple upsampling -- this assumes that the incoming sampling rate is less than the audio rate, which it might not be:
  // TODO generalize the resampler (and add filtering ?)
  int       upsample;               // flag to upsample or not
  int       sr_pd;                  // pd's sampling rate
  int       sr_lsl;
  double    sr_ratio;
  
  // containers for lsl api
  lsl_inlet               lsl_inlet_obj;      // instantiation of the inlet class
  lsl_streaminfo          lsl_info_list[50];  // to contain list returned by continuous resolver
  int                     lsl_info_list_cnt;
  int                     which;
  lsl_continuous_resolver lsl_cr;
  lsl_channel_format_t    type;
  float                   ts;
  double                  lsl_pull_timeout;
  double                  lag_lsl;
  double                  cnt_lsl;

  // threading variables for the listen thread and associated data
  MUTEX           listen_lock;
  TID             tid;
  //pthread_mutex_t listen_lock;
  //pthread_t       tid;
  int             stop_;
  int             can_launch_resolver;
  
}t_lsl_inlet_tilde;


// for launching the resolve threads
typedef struct _prop_strct{
  
  struct t_lsl_tilde_inlet *x;
  int          argc;
  t_atom      *argv;

}t_prop_strct;


// listen thread function declaration:
//static void *lsl_listen_thread(void *in);

// helper function declarations:
static void destroy_info_list(t_lsl_inlet_tilde *x);
static void post_info_list(t_lsl_inlet_tilde *x);
static int prop_resolve(t_lsl_inlet_tilde *x, int argc, t_atom *argv);
static void flush_lsl_buffers(t_lsl_inlet_tilde *x);
static void free_lsl_buffers(t_lsl_inlet_tilde *x);
static void setup_lsl_buffers(t_lsl_inlet_tilde *x);

/********spline interpolation*********/
float spline_interpolate(t_float *buffer, long bufferLength, double findex)
{
	long lindex = findex;
	t_sample fr = findex - lindex;

	float p0 = buffer[(lindex - 2) % bufferLength];
	float p1 = buffer[(lindex - 1) % bufferLength];
	float p2 = buffer[(lindex) % bufferLength];
	float p3 = buffer[(lindex + 1) % bufferLength];
	float p4 = buffer[(lindex + 2) % bufferLength];
	float p5 = buffer[(lindex + 3) % bufferLength];

	return p2 + 0.04166666666*fr*((p3 - p1)*16.0 + (p0 - p4)*2.0
		+ fr * ((p3 + p1)*16.0 - p0 - p2 * 30.0 - p4
			+ fr * (p3*66.0 - p2 * 70.0 - p4 * 33.0 + p1 * 39.0 + p5 * 7.0 - p0 * 9.0
				+ fr * (p2*126.0 - p3 * 124.0 + p4 * 61.0 - p1 * 64.0 - p5 * 12.0 + p0 * 13.0
					+ fr * ((p3 - p2)*50.0 + (p1 - p4)*25.0 + (p5 - p0)*5.0)))));
}

// perform forward decl:
static t_int *lsl_inlet_tilde_perform(t_int *w);

// listen thread function:
DWORD WINAPI lsl_listen_thread(void *in)
{
	//static void *lsl_listen_thread(void *in){


	t_lsl_inlet_tilde *x = (t_lsl_inlet_tilde *)in;

	int ec;
	int i;
	lsl_channel_format_t type;
	double ts;
	double *sample_d;
	float *sample_f;
	int *sample_i;

	post("listening");
	sample_d = (double *)t_getbytes(0);
	sample_d = (double *)t_resizebytes(sample_d, 0, sizeof(double)*x->nchannels);
	sample_f = (float *)t_getbytes(0);
	sample_f = (float *)t_resizebytes(sample_f, 0, sizeof(float)*x->nchannels);
	sample_i = (int *)t_getbytes(0);
	sample_i = (int *)t_resizebytes(sample_i, 0, sizeof(int)*x->nchannels);

	type = lsl_get_channel_format(x->lsl_info_list[x->which]);
	post("connecting to %s...", lsl_get_name(x->lsl_info_list[x->which]));
	x->lsl_inlet_obj = lsl_create_inlet(x->lsl_info_list[x->which], 300, 1, 1);
	if (x->lsl_inlet_obj != 0)
		post("successfully connected");
	else
	{
		post("could not establish lsl connection");
		return 0;
	}
	x->connected = 1;

	x->stop_ = 0;
	post("%d", type);
	x->cnt_lsl = 0.0;
	while (x->stop_ == 0) 
	{

		switch (type) 
		{

		case cft_float32:
			ts = lsl_pull_sample_f(x->lsl_inlet_obj, sample_f, x->nchannels, LSL_FOREVER, &ec);
			if (x->cnt_lsl <= x->lag_lsl)
				x->cnt_lsl += x->sr_ratio;
			//post("%d", x->widx);
			EnterCriticalSection(&x->listen_lock);
			for (i = 0; i < x->nchannels; i++)
			{
				
				x->sig_buffs[i][x->widx] = (t_sample)sample_f[i];
				//if (i == 0)
				//	post("%f", x->sig_buffs[i][x->widx]);
				//post("%f %d\n", sample_f[i], i);
			}
			x->ts_buf[x->widx++] = (t_sample)ts;
			x->ridx = x->widx - x->lag*x->sr_ratio;
			while (x->widx >= x->buflen)x->widx -= x->buflen;
			while (x->widx < 0)x->widx++;
			
			while (x->m_dReadIdx > x->buflen - 1)
				x->m_dReadIdx -= (double)x->buflen;
			while (x->m_dReadIdx < 0)
				x->m_dReadIdx += (double)x->buflen;
			
			//pthread_mutex_lock(&x->listen_lock);
			LeaveCriticalSection(&x->listen_lock);


			//pthread_mutex_unlock(&x->listen_lock);

			break;

		case cft_double64:
			break;

		case cft_int32:
			break;


		}

	}

	t_freebytes(sample_d, sizeof(double)*x->nchannels);
	t_freebytes(sample_f, sizeof(float)*x->nchannels);
	t_freebytes(sample_i, sizeof(int)*x->nchannels);
	x->connected = 0;
	return 0;//;NULL;
}



// helper functions:
void destroy_info_list(t_lsl_inlet_tilde *x){

  int i;

  for(i=0;i<50;i++){
    if(x->lsl_info_list[i] != NULL){
      lsl_destroy_streaminfo(x->lsl_info_list[i]);
      x->lsl_info_list[i] = NULL;
    }
  }
  x->lsl_info_list_cnt=0;
}

void post_info_list(t_lsl_inlet_tilde *x){
  
  int i;
  int cnt = (x->lsl_info_list_cnt>=50)?50:x->lsl_info_list_cnt;
  post("----------available lsl streams------------");
  for(i=0;i<cnt;i++){
    post("[%d] name: %s  |  type: %s  |  source_id: %s",
    	 i,
    	 lsl_get_name(x->lsl_info_list[i]),
    	 lsl_get_type(x->lsl_info_list[i]),
    	 lsl_get_source_id(x->lsl_info_list[i]));
  }
}

int prop_resolve(t_lsl_inlet_tilde *x, int argc, t_atom *argv){

  t_symbol *arg;
  int resolved_count = 0;
  int i;
  t_atom *outv;

  post("prop_resolve x: %d, argc %d, argv %d", x, argc, argv);
  
  for(i=0;i<50;i++){
    if(x->lsl_info_list[i] == NULL)break;
    else{
      lsl_destroy_streaminfo(x->lsl_info_list[i]);
      x->lsl_info_list[i] = NULL;
    }
  }
  
  x->lsl_info_list_cnt = 0;
  arg = atom_getsymbolarg(0, argc, argv);
  if(!strcmp(arg->s_name, "-name"))
      resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "name", atom_getsymbolarg(1,argc,argv)->s_name,0,5);

  else if(!strcmp(arg->s_name, "-type"))
      resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "type", atom_getsymbolarg(1,argc,argv)->s_name,0,5);

  else if(!strcmp(arg->s_name, "-source_id"))
      resolved_count = lsl_resolve_byprop(x->lsl_info_list, 50, "source_id", atom_getsymbolarg(1,argc,argv)->s_name,0,5);

  else
    pd_error(x, "lsl_inlet: %s: uniknown flag or argument missing", arg->s_name);

  if(resolved_count!=0)
      return(resolved_count);
  else{
    post("could not find any streams of property %s matching value %s", arg->s_name, atom_getsymbolarg(1,argc,argv)->s_name);
    return 0;
  }
}

// pd methods:
void lsl_inlet_disconnect(t_lsl_inlet_tilde *x){
  
  if(x->stop_!=1){
    post("disconnecting from %s stream %s (%s)...",
	 lsl_get_type(x->lsl_info_list[x->which]),
    	 lsl_get_name(x->lsl_info_list[x->which]),
    	 lsl_get_source_id(x->lsl_info_list[x->which]));
    x->stop_=1;
    x->which = -1;
    // here we are forced to call the dreaded pthread_cancel
    // because the lsl_inlet waits forever for a new sample to come in
    // if a stream disappears before the thread exits, it will simply
    // stick waiting for a new sample and joining the thread will
    // halt pd in that state
    //pthread_cancel(x->tid);
    // however, we manage all of our resources outside of that thread, so it's cool (I think...)
    
    if(x->lsl_inlet_obj!=NULL){
      lsl_destroy_inlet(x->lsl_inlet_obj);
      x->lsl_inlet_obj=NULL;
    }
    post("...disconnected");
    x->connected = 0;
    x->ready = 0;
  }
}

void lsl_inlet_connect_by_idx(t_lsl_inlet_tilde *x, t_floatarg f)
{

	int ec;
	float fL, fpLplus;

	if (x->lsl_info_list_cnt == 0) 
	{
		post("No lsl_info objects available. Please try to resolve available LSL outlets.");
		return;
	}
	else if ((f > x->lsl_info_list_cnt) || (f < 0)) 
	{
		post("Invalid selection from list of available outlets.");
		return;
	}
	else 
	{

		x->which = (int)f;
		post("connecting to %s stream %s (%s)...",
			lsl_get_type(x->lsl_info_list[x->which]),
			lsl_get_name(x->lsl_info_list[x->which]),
			lsl_get_source_id(x->lsl_info_list[x->which]));
		//if(x->lsl_inlet_obj!=NULL)lsl_destroy_inlet(x->lsl_inlet_obj);

		if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_double64)
			if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_float32)
				if (lsl_get_channel_format(x->lsl_info_list[x->which]) != cft_int32) 
				{
					pd_error(x, "requested stream has invalid channel format, only floats, doubles, and 32-bit int data allowed");
					return;
				}
		if (lsl_get_nominal_srate(x->lsl_info_list[x->which]) == 0) 
		{
			pd_error(x, "requested stream has invalid nominal sampling rate, this must not be 0");
			return;
		}

		// prepare the ring buffers based on the stream info
		x->nchannels = lsl_get_channel_count(x->lsl_info_list[x->which]);
		x->longbuflen = x->nchannels * x->buflen;
		//post("%d", x->nchannels);
		if (x->nchannels > x->nout) 
		{
			pd_error(x, "the requested stream has more channels than are available in this pd object");
			return;
		}
		setup_lsl_buffers(x);

		// prepare the upsampling factors based on the stream info
		x->sr_lsl = lsl_get_nominal_srate(x->lsl_info_list[x->which]);
		x->sr_ratio = (double)x->sr_lsl / (double)x->sr_pd;
		x->lag_lsl = x->sr_ratio*(double)x->lag;
		x->m_dReadIdx = 0.0;
		

		post("...connected, launcing listener thread");
		x->tid = 0;
		x->tid = CreateThread(NULL, 0, lsl_listen_thread, (void *)x, 0, NULL);
		//ec = pthread_create(&x->tid, NULL, lsl_listen_thread, (void *)x);
		if (x->tid == 0) 
		{
			pd_error(x, "Error launching listener thread");
			return;
		}
	}
}

// this works:
DWORD WINAPI list_all_thread(void* in){ 
//void *list_all_thread(void *in){
  
  t_lsl_inlet_tilde *x = (t_lsl_inlet_tilde *)in;
  int listed_count;
  post("Attempting to find LSL outlets on the network...");
  listed_count = lsl_resolve_all(x->lsl_info_list, 50, 5);
  if(listed_count!=0){
    x->lsl_info_list_cnt = (listed_count>50?50:listed_count);
    post_info_list(x);
  }
  else post("no streams available");
  
  x->can_launch_resolver = 1;
  return 0;// NULL;
}
void lsl_inlet_list_all(t_lsl_inlet_tilde *x){
  
  int ec;
  TID tid = 0;
 // pthread_t tid;
  if(x->can_launch_resolver == 0)
    post("LSL outlets cannot be listed at this time. Another process is already at work.");
  else{
    x->can_launch_resolver = 0; // weak locking
	tid = CreateThread(NULL, 0, list_all_thread, &x, 0, NULL);
    //ec = pthread_create(&tid, NULL, list_all_thread, (void *)x);
    if(tid==0){
      pd_error(x, "Error launching list all thread");
      return;
    }
  }
}

// but for some reason this doesn't work:
/* void *list_by_property_thread(void *in){ */

/*   t_prop_strct *y = (t_prop_strct *)in; */
/*   int listed_count; */
/*   post("thread x: %d, argc %d, argv %d", y->x, y->argc, y->argv); */
/*   listed_count = prop_resolve(y->x, y->argc, y->argv); */
/*   if(listed_count!=0){ */
/*     y->x->lsl_info_list_cnt = (listed_count>50?50:listed_count); */
/*     post_info_list(y->x); */
/*   } */
 
/*   y->x->can_launch_resolver = 1; */
/*   return NULL; */
/* } */
void lsl_inlet_list_by_property(t_lsl_inlet_tilde *x, t_symbol *s, int argc, t_atom *argv){

    int listed_count;
    //post("thread x: %d, argc %d, argv %d", y->x, y->argc, y->argv);
  listed_count = prop_resolve(x, argc, argv);
  if(listed_count!=0){
    x->lsl_info_list_cnt = (listed_count>50?50:listed_count);
    post_info_list(x);
  }

  
}

void lsl_inlet_resolve_by_property(t_lsl_inlet_tilde *x, t_symbol *s, int argc, t_atom *argv){

  x->lsl_info_list_cnt = 0;
  x->lsl_info_list_cnt = prop_resolve(x, argc, argv);
  if(x->lsl_info_list_cnt != 0)
    lsl_inlet_connect_by_idx(x, 0);
  
}



t_int *lsl_inlet_tilde_perform(t_int *w)
{

	int i, n, rd_pt;
	t_lsl_inlet_tilde *x = (t_lsl_inlet_tilde *)(w[1]);
	t_sample *lcl_ts_out;
	t_sample* lcl_out;
	int sample_idx = 0;
	double dReadIdx;



	for (i = 0; i < x->nout; i++);
	lcl_ts_out = (t_sample *)w[i + 2];
	n = w[i + 3];

	//post("tilde perform: ready = %d, lag = %d, connected = %d", x->ready, x->lag, x->connected);
	if (x->connected == 1 && x->cnt_lsl >= x->lag_lsl)
	{

		EnterCriticalSection(&x->listen_lock);
		//post("------------------------------------------");
		//post("tilde perform: cnt_lsl = %f, lag_lsl = %f, readIdx = %f", x->cnt_lsl, x->lag_lsl, x->m_dReadIdx);
		dReadIdx = x->m_dReadIdx;
		while (n--)
		{
			for (i = 0; i < x->nout; i++)
			{
				lcl_out = (t_sample*)w[i + 2];
				*(lcl_out + sample_idx) = spline_interpolate(x->sig_buffs[i], x->buflen, dReadIdx);//1.0;//actual output
			}
			*(lcl_ts_out + sample_idx++) = (x->ts_buf, x->buflen, dReadIdx);//0.0;//actual output
			dReadIdx += x->sr_ratio;
			while (dReadIdx > x->buflen - 1)
				dReadIdx -= (double)x->buflen;
			while (dReadIdx < 0)
				dReadIdx += (double)x->buflen;



		}
		LeaveCriticalSection(&x->listen_lock);

	}
	return w + x->nout + 4;
}


void lsl_inlet_tilde_dsp(t_lsl_inlet_tilde *x, t_signal **sp)
{

	switch (x->nout) 
	{
	case 1:
		dsp_add(lsl_inlet_tilde_perform, 4, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[0]->s_n);
		break;
	case 2:
		dsp_add(lsl_inlet_tilde_perform, 5, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[2]->s_vec,
			sp[0]->s_n);
		break;
	case 4:
		dsp_add(lsl_inlet_tilde_perform, 7, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[2]->s_vec,
			sp[3]->s_vec,
			sp[4]->s_vec,
			sp[0]->s_n);
		break;
	case 8:
		dsp_add(lsl_inlet_tilde_perform, 11, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[2]->s_vec,
			sp[3]->s_vec,
			sp[4]->s_vec,
			sp[5]->s_vec,
			sp[6]->s_vec,
			sp[7]->s_vec,
			sp[8]->s_vec,
			sp[0]->s_n);
		break;
	case 16:
		dsp_add(lsl_inlet_tilde_perform, 19, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[2]->s_vec,
			sp[3]->s_vec,
			sp[4]->s_vec,
			sp[5]->s_vec,
			sp[6]->s_vec,
			sp[7]->s_vec,
			sp[8]->s_vec,
			sp[9]->s_vec,
			sp[10]->s_vec,
			sp[11]->s_vec,
			sp[12]->s_vec,
			sp[13]->s_vec,
			sp[14]->s_vec,
			sp[15]->s_vec,
			sp[16]->s_vec,
			sp[0]->s_n);
		break;
	case 32:
		dsp_add(lsl_inlet_tilde_perform, 35, x,
			sp[0]->s_vec,
			sp[1]->s_vec,
			sp[2]->s_vec,
			sp[3]->s_vec,
			sp[4]->s_vec,
			sp[5]->s_vec,
			sp[6]->s_vec,
			sp[7]->s_vec,
			sp[8]->s_vec,
			sp[9]->s_vec,
			sp[10]->s_vec,
			sp[11]->s_vec,
			sp[12]->s_vec,
			sp[13]->s_vec,
			sp[14]->s_vec,
			sp[15]->s_vec,
			sp[16]->s_vec,
			sp[17]->s_vec,
			sp[18]->s_vec,
			sp[19]->s_vec,
			sp[20]->s_vec,
			sp[21]->s_vec,
			sp[22]->s_vec,
			sp[23]->s_vec,
			sp[24]->s_vec,
			sp[25]->s_vec,
			sp[26]->s_vec,
			sp[27]->s_vec,
			sp[28]->s_vec,
			sp[29]->s_vec,
			sp[30]->s_vec,
			sp[31]->s_vec,
			sp[32]->s_vec,
			sp[0]->s_n);
		break;
	}
}

void flush_lsl_buffers(t_lsl_inlet_tilde *x)
{

	int i, j;
	if (x->sig_buffs != 0)
		for (i = 0; i < x->nout; i++)
			memset(x->sig_buffs[i], 0.0, x->buflen * sizeof(t_sample));
	/* for(i=0;i<x->buflen*x->nchannels;i++) */
	/*   x->sig_buf[i] =  0.0; */
	if (x->ts_buf != 0)memset(x->ts_buf, 0.0, x->buflen * sizeof(t_sample));
	/* for(i=0;i<x->buflen*x->nchannels;i++) */
	/*   x->ts_buf[i] = 0.0; */

	x->widx = 0;
	x->ridx = 0;
	x->ready = 0;

}
void free_lsl_buffers(t_lsl_inlet_tilde *x)
{

	int i;
	post("in the free method");
	if (x->sig_buffs != 0) 
	{
		post("inside sig_buffs");
		for (i = 0; i < x->nout; i++)
			if (x->sig_buffs[i] != 0)
				t_freebytes(x->sig_buffs[i], x->buflen*sizeof(float));
		t_freebytes(x->sig_buffs, x->nout * sizeof(t_sample*));
	}
	if (x->ts_buf != 0)
		t_freebytes(x->ts_buf, x->buflen * sizeof(float));

}

void setup_lsl_buffers(t_lsl_inlet_tilde *x)
{

	int i;
	post("%f", x->sig_buffs);
	if (x->sig_buffs != 0)free_lsl_buffers(x);

	x->sig_buffs = (t_sample **)t_getbytes(0);
	x->sig_buffs = (t_sample **)t_resizebytes(x->sig_buffs, 0, sizeof(t_sample*) * x->nout);
	for (i = 0; i < x->nout; i++)
	{
		x->sig_buffs[i] = (t_sample *)t_getbytes(0);
		x->sig_buffs[i] = (t_sample *)t_resizebytes(x->sig_buffs[i], 0, sizeof(float) * x->buflen);
	}

	x->ts_buf = (t_sample *)t_getbytes(0);
	x->ts_buf = (t_sample *)t_resizebytes(x->ts_buf, 0, sizeof(float) * x->buflen);
	//flush_lsl_buffers(x);
}

void *lsl_inlet_tilde_new(t_symbol *s, int argc, t_atom *argv)
{

	int i, lcl_nout;
	t_symbol *firstarg;
	t_lsl_inlet_tilde *x = (t_lsl_inlet_tilde *)pd_new(lsl_inlet_tilde_class);

	// defaults
	x->nout = 8;
	x->buflen = 10 * sys_getblksize();
	x->connected = 0;

	x->widx = 0;
	x->ridx = 0;
	x->ready = 0;
	x->lag = sys_getblksize();
	x->sr_pd = sys_getsr();
	x->sr_ratio = 1.0;
	x->lag_lsl = (double)x->lag;

	// parse creation args
	while (argc > 0) {
		firstarg = atom_getsymbolarg(0, argc, argv);
		if (!strcmp(firstarg->s_name, "-lag")) 
		{

			x->lag = sys_getblksize() * atom_getfloatarg(1, argc, argv);
			x->lag_lsl = (double)x->lag;
			argc -= 2;
			argv += 2;
		}

		else if (!strcmp(firstarg->s_name, "-buflen")) 
		{
			if (atom_getfloatarg(1, argc, argv) == 0)
				x->buflen = 1;
			else 
			{
				x->buflen = sys_getblksize() * atom_getfloatarg(1, argc, argv);
				argc -= 2;
				argv += 2;
			}
		}

		else if (!strcmp(firstarg->s_name, "-nout")) 
		{
			lcl_nout = (atom_getfloatarg(1, argc, argv));
			if ((lcl_nout == 1) || (lcl_nout == 2) || (lcl_nout == 4) ||
				(lcl_nout != 8) || (lcl_nout != 16) || (lcl_nout != 32)) 
			{

				x->nout = lcl_nout;

			}
			else
				post("Invalid outlet selection (must be 1, 2, 4, 8, 16, or 32): reverting to 8");

			argc -= 2;
			argv += 2;
		}
		else 
		{
			pd_error(x, "%s: unkown flag or argument missing",
				firstarg->s_name);
			argc--, argv++;
		}
	}
	

	// allocate these once we connect to the outlet
	x->sig_buffs = 0;
	//for (i = 0; i < x->nout; i++)
		//x->sig_buffs[i] = 0;
	x->ts_buf = 0;
	x->nchannels = 0; // this gets set on inlet creation

	// setup the memory
	x->lcl_outs = (t_sample **)t_getbytes(0);
	x->lcl_outs = (t_sample **)t_resizebytes(x->lcl_outs, 0, sizeof(t_sample *) * x->nout);

	// setup outlets
	x->sig_outlets = (t_outlet **)t_getbytes(0);
	x->sig_outlets = (t_outlet **)t_resizebytes(x->lcl_outs, 0, sizeof(t_sample *) * x->nout);
	for (i = 0; i < x->nout; i++) x->sig_outlets[i] = outlet_new(&x->x_obj, &s_signal);
	x->ts_outlet = outlet_new(&x->x_obj, &s_signal);

	x->which = -1;
	x->can_launch_resolver = 1;


	for (i = 0; i < 50; i++)
		x->lsl_info_list[i] = NULL;
	x->lsl_info_list_cnt = 0;
	//x->lsl_inlet_obj = NULL;

	//x->listen_lock = PTHREAD_MUTEX_INITIALIZER;
	InitializeCriticalSection(&x->listen_lock);
	x->stop_ = 1;

	return x;

}



void lsl_inlet_tilde_free(t_lsl_inlet_tilde *x)
{

	int i;
	DeleteCriticalSection(&x->listen_lock);

	destroy_info_list(x);
	lsl_inlet_disconnect(x);
	if (x->lsl_inlet_obj != NULL)lsl_destroy_inlet(x->lsl_inlet_obj);

	if (x->lcl_outs != 0)
		t_freebytes(x->lcl_outs, sizeof(t_sample *)*x->nout);

	if (x->sig_outlets != 0)
		t_freebytes(x->sig_outlets, sizeof(t_outlet *)*x->nout);

	free_lsl_buffers(x);

}

void lsl_inlet_tilde_setup(void)
{

	lsl_inlet_tilde_class = class_new(gensym("lsl_inlet~"),
		(t_newmethod)lsl_inlet_tilde_new,
		(t_method)lsl_inlet_tilde_free,
		sizeof(t_lsl_inlet_tilde),
		0,
		A_GIMME,
		0);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_disconnect,
		gensym("disconnect"),
		0);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_connect_by_idx,
		gensym("connect_by_idx"),
		A_DEFFLOAT,
		A_NULL);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_list_all,
		gensym("list_all"),
		A_NULL);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_list_by_property,
		gensym("list_by_property"),
		A_GIMME,
		A_NULL);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_resolve_by_property,
		gensym("resolve_by_property"),
		A_GIMME,
		A_NULL);

	class_addmethod(lsl_inlet_tilde_class,
		(t_method)lsl_inlet_tilde_dsp, gensym("dsp"), A_NULL);

	/* class_addbang(lsl_inlet_class, */
	/* 		(t_method)lsl_inlet_bang); */

}









