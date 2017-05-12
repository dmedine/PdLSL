#include "m_pd.h"
#include "lsl_c.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>



/**************** lsl inlet ****************/

static t_class *lsl_inlet_class;

typedef struct _lsl_inlet{

  t_object    x_obj;

  char**      str_message_buffer;   // only allocate for this if the channel format is string
  float*      f_message_buffer;     // this is for all other valid channel formats (the outlet is s_list regardless)
  float*      timestamp_buffer;
  int         max_hist;
  int         max_len;
  int         current_count;        // we need this to know if the buffer is full or not

  // containers for lsl api
  lsl_inlet   lsl_inlet;
  int         max_buflen;
  int         max_chunlen;
  
}t_lsl_inlet;

void lsl_inlet_data_allocate(t_lsl_inlet *x, lsl_channel_format_t channel_format);
int lsl_inlet_parse_creation_args(t_lsl_inlet *x, int argc, t_atom *argv);


// helper functions:
void lsl_inlet_data_allocate(t_lsl_inlet *x, lsl_channel_format_t channel_format){

  int i;

  // allocate memory for the message buffers
  if(channel_format == cft_string){
    x->str_message_buffer = (char **)malloc(sizeof(char *) * x->max_hist);
    for(i=0;i<x->max_len;i++)
      x->str_message_buffer[i] = (char *)malloc(sizeof(char) * x->max_len);
  }
  else
    x->f_message_buffer = (float *)malloc(sizeof(float) * x->max_hist);
  
  x->timestamp_buffer = (float *)malloc(sizeof(float) * x->max_hist);

}

int lsl_inlet_parse_creation_args(t_lsl_inlet *x, int argc, t_atom *argv){

  int res = 0;

  // data containers for lsl_stream_info object:
  char stream_name[100];
  char stream_type[100];
  char stream_format[100];
  int srate = 0;                            // trigger channel
  lsl_channel_format_t channel_format = -1;
  char source_id[100];
  lsl_streaminfo info;

  strcpy(stream_name, "\n");
  strcpy(stream_format, "\n");              // this is all that is allowed here
  strcpy(source_id, "\n");                  // this is optional, but needed in the c api so default is empty
  strcpy(stream_type, "pd_lsl");            // also optional
  
  while(argc > 0) {
    t_symbol *arg = atom_getsymbolarg(0, argc, argv);
    if(!strcmp(arg->s_name, "-name"))
      {
	strcpy(stream_name, atom_getsymbolarg(1, argc, argv)->s_name);
	argc-=2, argv+=2;
      }

    if(!strcmp(arg->s_name, "-type"))
      {
	strcpy(stream_type, atom_getsymbolarg(1, argc, argv)->s_name);
	argc-=2, argv+=2;
      }
    
    if(!strcmp(arg->s_name, "-format"))
      {
	strcpy(stream_format, atom_getsymbolarg(1, argc, argv)->s_name);
	argc-=2, argv+=2;
      }
    
    if(!strcmp(arg->s_name, "-id"))
      {
	strcpy(source_id, atom_getsymbolarg(1, argc, argv)->s_name);
	argc-=2, argv+=2;
      }
    
    if(!strcmp(arg->s_name, "-max_len"))
      {
	x->max_len = (int)atom_getfloatarg(1, argc, argv);
	argc-=2, argv+=2;
      }
	
    if(!strcmp(arg->s_name, "-max_hist"))
      {
	x->max_hist = (int)atom_getfloatarg(1, argc, argv);
	argc-=2, argv+=2;
      }

    if(!strcmp(arg->s_name, "-max_buflen"))
      {
	x->max_hist = (int)atom_getfloatarg(1, argc, argv);
	argc-=2, argv+=2;
      }
 
    argc--,argv++;
  }


  if(strcmp(stream_type, "\n")){
    if(!strcmp(stream_format, "cft_float32"))channel_format = cft_float32;
    if(!strcmp(stream_format, "cft_double64"))channel_format = cft_double64;
    if(!strcmp(stream_format, "cft_string"))channel_format = cft_string;
    if(!strcmp(stream_format, "cft_int32"))channel_format = cft_int32;
    if(!strcmp(stream_format, "cft_int16"))channel_format = cft_int16;
    if(!strcmp(stream_format, "cft_int8"))channel_format = cft_int8;
    if(!strcmp(stream_format, "cft_int64"))channel_format = cft_int64;
  }

  // we have enough input to form the inlet
  if(!strcmp(stream_name,"\n") && channel_format != -1){
    // setup the memory on pd's side
    lsl_inlet_data_allocate(x, channel_format);
    info = lsl_create_streaminfo(stream_name, stream_type, 1, 0, channel_format, source_id);
    x->lsl_inlet = lsl_create_inlet(info, x->max_buflen, 0, 1);
    res = 1;
  }

  return res;
}


static void *lsl_inlet_new(t_symbol *s, int argc, t_atom *argv){

  t_lsl_inlet *x = (t_lsl_inlet *)pd_new(lsl_inlet_class);
  int got_inlet;
  
  // default values for the message_buffer
  x->max_len = 100;
  x->max_hist = 100;

  // defaults for the inlet
  x->max_buflen = 100; // 10000 samples

  x->str_message_buffer = 0;
  x->f_message_buffer = 0;
  x->timestamp_buffer = 0;
  
  got_inlet = lsl_inlet_parse_creation_args(x, argc, argv);
    

  return x;
    
}

static void lsl_inlet_free(t_lsl_inlet *x){

  int i;
  if(!x->str_message_buffer) {
    for(i=0;i<x->max_hist;i++)
      free(x->str_message_buffer[i]);
    free(x->str_message_buffer);
  }
  if(!x->f_message_buffer)free(x->f_message_buffer);
  if(!x->timestamp_buffer)free(x->timestamp_buffer);
}

void lsl_inlet_setup(void){

  lsl_inlet_class = class_new(gensym("lsl_inlet"),
			      (t_newmethod)lsl_inlet_new,
			      (t_method)lsl_inlet_free,
			      sizeof(t_lsl_inlet),
			      0,
			      A_GIMME,
			      0);

}
