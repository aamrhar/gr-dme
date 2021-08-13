/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "dme_tx_impl.h"

namespace gr {
  namespace dme {

    dme_tx::sptr
    dme_tx::make(float samp_rate, float vor_freq,int id_num)
    {
      return gnuradio::get_initial_sptr
        (new dme_tx_impl(samp_rate, vor_freq,id_num));
    }

    /*
     * The private constructor
     */
    dme_tx_impl::dme_tx_impl(float samp_rate, float vor_freq,int id_num)
      : gr::sync_block("dme_tx",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(float)))
    {
       message_port_register_out(pmt::mp("cmd_out"));
       message_port_register_in(pmt::mp("from_rx"));
       set_msg_handler(pmt::mp("from_rx"),
                       boost::bind(&dme_tx_impl::handle_rx,
                                   this, _1));
        /****************
         * Wave gen
         * *************/

        std::cout <<std::setprecision(PRECISION) <<std::endl <<
        std::endl <<
        "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl <<
        "++" << std::endl <<
        "++ Initialization of LASSENA - DME subsystem" << std::endl <<
        "++" << std::endl <<
        "++ - Initialization of random seed...";
        srand(time(NULL));
        std::cout << " Done." << std::endl;

        std::cout << "++" << std::endl<< "++ - Resetting parameters...";

        // State Variables
        d_enable_ctrl = false;

        std::stringstream ss;
        ss<<"DME"<<id_num;
        c_dme_id = pmt::intern(ss.str());

        d_samp_rate = samp_rate;
        Set_Mode(vor_freq);

        d_Op_Mode = Search_Mode;

        d_Jitter_Count = 0;
        d_fs = samp_rate/MHZ;

        d_Sample_Count = 0;
        d_Pulse_Count = 0;
        d_Max_Pulse = MAX_PULSES;
        d_Time = 0;
        d_Max_Occur = 0;
        d_N_Measurements = 0;
        d_Jitter_Count = 0;
        d_Threshold = 0.05; //TODO: why not

        d_counter =0;

        d_Search_Min = (int) ceil(d_samp_rate / PPS_SEARCH); // samples
        d_Search_Range =  (int) floor(0.5 * d_Search_Min); // samples
        d_Track_Min = (int) ceil(d_samp_rate / PPS_TRACK); // samples
        d_Track_Range =  (int) floor(0.5 * d_Track_Min); // samples
        d_Wave_Counter = genwave();
        repport_mode();

        // Vectors
        d_Trans_Jitter = std::vector<int>();
        d_Time_Measurements = std::vector<float>();
        d_Occurrences = std::vector<int>();
        d_Meas_index = 0;

        std::cout << " Done." << std::endl <<
         "++" << std::endl;
        std::cout <<
        "++ - Initialization finished." << std::endl <<
        "++" << std::endl <<
        "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    }

    /*
     * Our virtual destructor.
     */
    dme_tx_impl::~dme_tx_impl()
    {
        std::cout<<"DMEX stopped"<<std::endl;
    }
    /***************************************************
     * Private Methods
     * ***********************************************/
    int dme_tx_impl::genwave()
    {
        std::cout << "++" << std::endl <<
        "++ - Generating pair-of-pulses waveform...";
        // First from t = 0 to Delay/2 + Guard
        float MaxT = 0;
        float fs = d_samp_rate/MHZ;
        if (d_Mode==XMode) { // Mode X
        MaxT = XMODE_DELAY_RX * 0.5 + MODE_DELAY_GUARD;
        }
        else {
        MaxT = YMODE_DELAY_RX * 0.5 + MODE_DELAY_GUARD;
        }

        d_Wave.clear();
        for(float t = 0; t <= MaxT; t+=1/fs) {
        if (d_Mode == XMode) { // Mode X
          d_Wave.push_back(
                exp(-pow(t - XMODE_DELAY_RX * 0.5, 2) * 0.5 * PULSE_VAR)
              + exp(-pow(t + XMODE_DELAY_RX * 0.5, 2) * 0.5 * PULSE_VAR) );
        }
        else {
          d_Wave.push_back(
                exp(-pow(t - YMODE_DELAY_RX * 0.5, 2) * 0.5 * PULSE_VAR)
              + exp(-pow(t + YMODE_DELAY_RX * 0.5, 2) * 0.5 * PULSE_VAR) );
        }
        }
        d_Wave_Samples = d_Wave.size();
        // Now replicate for negative time (-Delay/2-Guard to 0)
        for(int i = 1; i < d_Wave_Samples; i++) {
        d_Wave.insert(d_Wave.begin(), d_Wave[2*i-1]);
        }
        d_Wave_Samples = d_Wave.size();
        std::cout << " Done." << std::endl <<
        "++   - Loaded " << std::dec << d_Wave_Samples << " samples." << std::endl;

        /*
        * Debug:  Wave[k]
        *
        for(int k = 0; k < d_Wave_Samples; k++ ) {
        std::cout << std::setprecision(4) << d_Wave[k] << " ";
        }
        /***/

        return d_Wave_Samples;
    }
    void
    dme_tx_impl::repport_mode()
    {
        pmt::pmt_t pmt_mode = (d_Mode==XMode) ? pmt::intern("X"):pmt::intern("Y");
        message_port_pub(pmt::mp("cmd_out"),
                         pmt::list3(
                             c_dme_id,
                             pmt::intern("MODE"),
                             pmt_mode));
    }

    /***************************************************
     * Public Methods
     * ***********************************************/
    //Message handler
    void
    dme_tx_impl::handle_rx(pmt::pmt_t msg)
    {
        if(pmt::is_integer(msg))
        {
            int cmd=pmt::to_long(msg);
            switch (cmd) {
            case MODEX_CMD:
                d_Mode =XMode;
                d_Mode_Delay=XMODE_DELAY_RX;
                d_Wave_Counter = genwave();
                repport_mode();
                break;
            case MODEY_CMD:
                d_Mode =YMode;
                d_Mode_Delay=YMODE_DELAY_RX;
                d_Wave_Counter = genwave();
                repport_mode();
                break;
            case SEARCH_CMD:
                d_Op_Mode = Search_Mode;
                break;
            case TRACK_CMD:
                d_Op_Mode = Tracking_Mode;
                break;
            default:
                break;
            }
        }
    }
    //Accessors
    void
    dme_tx_impl::Set_Mode(float vor_freq)
    {
        if ( (vor_freq - floor(vor_freq*10)/10) == 0.00 ){ // X Mode
            d_Mode = XMode;
            d_Mode_Delay = XMODE_DELAY_RX ;
            std::cout<<"XMODE"<<std::endl;
            //cout << setprecision(8) << " vor_freq  "<< vor_freq << endl;
            //cout << "floor(vor_freq*10)/10 =      " << floor(vor_freq*10)/10 << endl;

        }else if ( abs(vor_freq - floor(vor_freq*10)/10 - 0.05) < 1e-5){ // Y Mode
            d_Mode = YMode;
            d_Mode_Delay = YMODE_DELAY_RX; //NOTE: changed from 30 to 36
            std::cout<<"YMODE"<<std::endl;
        }
        d_Wave_Counter = genwave();
        repport_mode();
    }
    /* *************************************************
     * WORK!
     * ************************************************/
    int
    dme_tx_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      float* out = (float *) output_items[0];
      for(int i = 0; i <  noutput_items ; i++)
      {
          if(d_Jitter_Count > 0)
          {
              --d_Jitter_Count;
          }else // Transmit new pulse
          {
              // Start transmitting Wave
              d_Wave_Counter = 0;
              d_counter++;
               //Set the new random delay
              if(d_Op_Mode==Search_Mode)
              {
                  d_Jitter_Count = rand() % d_Search_Range + d_Search_Min;
              }else
              {
                  d_Jitter_Count = rand() % d_Track_Range + d_Track_Min;
              }
          }
          // Set the output
          if(d_Wave_Counter < d_Wave_Samples)  // Still transmitting
          {
              // Output
              out[i] = d_Wave[d_Wave_Counter];
              if (  d_Wave_Counter == (d_Wave_Samples-7))
              {
                  d_counter_pulses++;
              }
              // Increase Wave index
              ++d_Wave_Counter;
          } else // Waiting for the next pulse
          {
              out[i] = 0;
          }
      } // for(int i = 0; i <  noutput_items ; i++)



      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace dme */
} /* namespace gr */

