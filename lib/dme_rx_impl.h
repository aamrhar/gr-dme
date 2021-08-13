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

#ifndef INCLUDED_DME_DME_RX_IMPL_H
#define INCLUDED_DME_DME_RX_IMPL_H

#include <dme/dme_rx.h>
#include <string>

namespace gr {
  namespace dme {

    class dme_rx_impl : public dme_rx
    {
     private:
        float* d_Zero_Cross_p;
        float* d_Not_Zero_Cross_p;
        int d_history_samples;
        float* d_constant_4_vector_p;
        //float *constant_0_vector;
        char* d_Detection_p;
        char* d_Not_Detection_p;
        char* d_Detection_in_threshold_p;
        float* d_Detection_in_threshold_float_p;
        char* d_Receive_Jitter_p;
        float* d_Threshold_vector_p;
        float* d_in_times_four_p;
        float d_vor_freq;
        float d_old_vor_freq;
        int d_counter_rx_pulses;
        std::vector<long int> d_Received_Jitters_v;
        int d_counter_tx_pulses;
        std::vector<long int> d_Transmitted_Jitters_v;
        std::vector<long int> d_In_Window_Jitters_v;
        long int d_last_tx_Jitter_rx_ok;
        int d_NSamples_WINDOW_LENGTH;
        int d_counter_recovered;
        int d_counter_recovered_for_cout;
        //bool d_lock; //TODO: RM
        int d_ndetections;
        int d_counter_for_print_warning;

        mutable boost::mutex mutex;

        std::vector< std::pair<int,int> > d_pairs_txseq_and_difference_v; //( Tx seq number, copy_of_nsamples_diff)
        int d_nsamples_diff;

        std::vector< std::pair<int,int> > d_pairs_number_and_ocurrence_v; //( Tx seq number, copy_of_nsamples_diff)

        std::pair<int,int> d_greater_occurrence_pr;

        XYModetype d_Mode;
        Op_Modetype d_Op_Mode;
        int d_Mode_Delay;
        int d_Detection_Delay;
        int d_first_positive_index;
        int d_first_negative_index;

        int d_samp_rate;
        std::string d_dme_id; /// string dme id
        std::string d_vor_lab; ///  dme vor label
        long d_bias; /// dme time offset

        /***************************************************
         * Private Methods
         * ***********************************************/

     public:

        /***************************************************
         * Public Methods
         * ***********************************************/
        //Message handlers
        /**
         * @brief tx_handler : records tx pulse tranmission time
         * @param msg : a long uint64 pmt
         *              that represents tranmission time in samples
         *  output //TODO: add output
         */
        void tx_handler(pmt::pmt_t msg);
        /**
         * @brief cmd_handler : handles commands for UI
         * @param msg expected pmt = list3(src,label,val)
         *      src:UI or UGC
         *      label : VORx (x=dme_id)
         *              BIAS
         */
        void cmd_handler(pmt::pmt_t msg);
        //callbacks
        /**
         * @brief set_mode sets mode (X/Y)
         * @param vor_freq vor frquency in MHz
         */
        void set_mode(float vor_freq);


        dme_rx_impl(float samp_rate, float freq_vor, int id_num);
        ~dme_rx_impl();

        // Where all the action really happens
        int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_DME_RX_IMPL_H */

