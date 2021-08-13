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

#ifndef INCLUDED_DME_DME_TX_IMPL_H
#define INCLUDED_DME_DME_TX_IMPL_H

#include <dme/dme_tx.h>
#include <string>

namespace gr {
  namespace dme {

    class dme_tx_impl : public dme_tx
    {
     private:
        XYModetype  d_Mode;
        Op_Modetype d_Op_Mode;
        int         d_Wave_Counter;// A counter used while transmitting Wave
        std::vector<float> d_Wave; // The pointer to the transmitted waveform (from file)
        int d_Wave_Samples; // The number of samples in Wave
        long int d_counter_pulses;
        int d_Mode_Delay;
        float d_samp_rate;

        bool d_enable_ctrl;//TODO: remove this?
        float d_fs;
        pmt::pmt_t c_dme_id; //dme id (or nav)
        //bool Mode; // DME mode. True for X-Mode

        int d_Sample_Count; // The number of samples since the last transmitted jitter - Bias (The true number of samples due to propagation)
        int d_Pulse_Count; // The number of pulses transmitted in the set (Search Mode) or missed replies (Track Modes)
        int d_Max_Pulse; // The current limit for Pulse_Count
        float d_Time; // The delay of the most frequent measurement
        int d_Max_Occur; // The number of occurrences of the most frequent
        int d_N_Measurements; // Number of measurements received after transmitted pulse
        float d_Threshold; // The adaptive detection threshold (Pulses' amplitude must exceed it)
        std::vector<int> d_Trans_Jitter; // A set of counters used to delay the jitter from TX to RX (by 'Bias' samples)
        std::vector<float> d_Time_Measurements; // The whole collection of TOAs within a set of pulses
        std::vector<int> d_Occurrences; // The number of occurrences associated with each measurement
        unsigned long int d_Meas_index; // To look for current measurement
        int d_Jitter_Count; // The counter used to introduce random delays
        /*
        * Initialized @ set_fs()
        */
        int d_Receive_Delay; // The number of samples between TOA and maximum amplitude
        int d_Max_Count; // The length of the window in samples
        float d_Tolerance; // The tolerance to determine a different delay measurement while in Search Mode (samples - float)
        float d_Track_Tolerance; // The tolerance to determine track loss (samples - float)
        int d_Search_Min; // Minimum delay between transmitted pair of pulses in Search mode (samples)
        int d_Search_Range; // Maximum-Minimum delay between transmitted pair of pulses in Search mode (samples)
        int d_Track_Min; // Minimum delay between transmitted pair of pulses in Track mode (samples)
        int d_Track_Range;// Maximum-Minimum delay between transmitted pair of pulses in Track mode (samples)

        float d_Predicted_Time; // Used in the calibration step to see if we can predict (within a sample) the delay and for detecting track lost in Track mode
        float d_Bias; // The delay in TX-to-RX path, as seen by the software
        int d_Bias_Inc; // The bias increment during calibration mode (samples)
        int d_Max_Delay; // The maximum delay allowed in TX-to-RX path (samples)

        int d_counter;
    /* ***************************************************
     *  Private methodes
     * *************************************************/
        /**
         * @brief genwave genarate wave from internal attributes
         *        in {d_samp_rate}
         *        out {d_wave,d_Wave_Samples}
         *
         * @return generated wave size (d_Wave_Samples)
         */
        int genwave();
        /**
         * @brief repport_mode : repport mode through msg
         */
        void repport_mode();

     public:
      dme_tx_impl(float samp_rate, float vor_freq,int id_num);
      ~dme_tx_impl();

      /***************************************************
       * Public Methods
       * ***********************************************/
      //Message handler
      /**
       * @brief rx message handler
       * @param a pmt message
       *        pmt should be long
       *        expected values:
       *            see RX-->TX commands in dme_constants.h
       */
      void handle_rx(pmt::pmt_t msg);
      //Callbacks
      /**
       * @brief Set_Mode: set dme mode (X or Y) from vor_freq
       *        set's d_Mode and d_Mode_Delay
       *
       * @param vor_freq VOR frequency in MHz
       */
      void Set_Mode(float vor_freq);

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_DME_TX_IMPL_H */

