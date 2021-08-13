/* -*- c++ -*- */

#define DME_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "dme_swig_doc.i"

%{
#include "dme/dme_tx.h"
#include "dme/dme_rx.h"
#include "dme/fpga_clk32_sim.h"
#include "dme/dme_time.h"
%}


%include "dme/dme_tx.h"
GR_SWIG_BLOCK_MAGIC2(dme, dme_tx);
%include "dme/dme_rx.h"
GR_SWIG_BLOCK_MAGIC2(dme, dme_rx);
%include "dme/fpga_clk32_sim.h"
GR_SWIG_BLOCK_MAGIC2(dme, fpga_clk32_sim);
%include "dme/dme_time.h"
GR_SWIG_BLOCK_MAGIC2(dme, dme_time);
