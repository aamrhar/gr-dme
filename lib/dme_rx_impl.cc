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
#include "dme_rx_impl.h"

#include <volk/volk.h>


namespace gr {
  namespace dme {

    dme_rx::sptr
    dme_rx::make(float samp_rate, float freq_vor, int id_num)
    {
      return gnuradio::get_initial_sptr
        (new dme_rx_impl(samp_rate, freq_vor, id_num));
    }

    /*
     * The private constructor
     */
    dme_rx_impl::dme_rx_impl(float samp_rate, float freq_vor, int id_num)
      : gr::sync_block("dme_rx",
              gr::io_signature::make2(2, 2, sizeof(float),sizeof(uint64_t)),
              gr::io_signature::make(1, 1, sizeof(float)))
    {
        // Message ports assignation
        message_port_register_out(pmt::mp("to_tx"));
        message_port_register_out(pmt::mp("dist"));

        message_port_register_in(pmt::mp("cmd_in"));
        set_msg_handler(pmt::mp("cmd_in"),
                        boost::bind(&dme_rx_impl::cmd_handler, this, _1));
        message_port_register_in(pmt::mp("tx_detector"));
        set_msg_handler(pmt::mp("tx_detector"),
                        boost::bind(&dme_rx_impl::tx_handler, this, _1));
        // init
        const int alignment_multiple = volk_get_alignment() / sizeof(float);
        set_alignment(std::max(1, alignment_multiple));

        //MEMORY Allocation for the floats to compute the zero crossing vector
        d_Zero_Cross_p = (float*)volk_malloc(sizeof(float)*16384,  alignment_multiple   );
        d_constant_4_vector_p= (float*)volk_malloc(sizeof(float)*16384,  alignment_multiple   );
        d_Threshold_vector_p= (float*)volk_malloc(sizeof(float)*16384,  alignment_multiple   );
        d_Detection_in_threshold_float_p =  (float*)volk_malloc(sizeof(float)*16384,  alignment_multiple   );
        d_in_times_four_p= (float*)volk_malloc(sizeof(float)*16384,  alignment_multiple   );
        //MEMORY Allocation for the chars representing logicals (8 bit bools) for detections
        d_Detection_p =  (char*)volk_malloc(sizeof(char)*16384,  alignment_multiple   );
        d_Not_Detection_p =  (char*)volk_malloc(sizeof(char)*16384,  alignment_multiple   );
        d_Detection_in_threshold_p =  (char*)volk_malloc(sizeof(char)*16384,  alignment_multiple   );
        d_Receive_Jitter_p = (char*)volk_malloc(sizeof(char)*16384,  alignment_multiple   );

        d_Transmitted_Jitters_v.clear();
        //cout << Transmitted_Jitters.size() << endl;
        d_Received_Jitters_v.clear();
        d_In_Window_Jitters_v.clear();
        //d_lock = false;

        for(int k = 0; k< 16384 ; k++)
        {
            d_constant_4_vector_p[k] = 4.0;
            //constant_0_vector[k] = 0.0;
            d_Threshold_vector_p[k] = MIN_THRESHOLD;
        }
        memset(d_Detection_p, 0x00,sizeof(char)*16384 );
        memset(d_Detection_in_threshold_p, 0x00,sizeof(char)*16384 );
        memset(d_Receive_Jitter_p, 0x00,sizeof(char)*16384 );

        //memset(Detection_in_threshold, 0x00,sizeof(char)*16384 );


        d_history_samples = 60;


        set_history(d_history_samples);

        d_counter_rx_pulses = 0;
        d_counter_tx_pulses = 0;
        d_NSamples_WINDOW_LENGTH =  (int) ceil(WINDOW_LENGTH*samp_rate/1e6);

        d_counter_recovered = 0;
        d_counter_recovered_for_cout  = 0;
        d_counter_for_print_warning = 0;

        d_vor_freq = freq_vor;
        // Mode = XMode;
        // Op_Mode = Search;second_positive_index
        d_samp_rate = samp_rate;
        d_old_vor_freq=0;
        set_mode(freq_vor);

        d_pairs_txseq_and_difference_v.clear();
        d_pairs_number_and_ocurrence_v.clear();
        d_last_tx_Jitter_rx_ok = 0;

        //for messages
        std::stringstream ss;
        ss<<"DME"<<id_num;
        d_dme_id =  ss.str();
        ss.str("");
        ss<<"VOR"<<id_num;
        d_vor_lab=ss.str();

        //for bias
        d_bias=0;

    }

    /*
     * Our virtual destructor.
     */
    dme_rx_impl::~dme_rx_impl()
    {
        std::cout<<d_dme_id<<" RX is stopped"<<std::endl;
    }

    /***************************************************
     * Public Methods
     * ***********************************************/
    //CALLBACK
    void
    dme_rx_impl::set_mode(float vor_freq)
    {
        int cmd;
        if ( (vor_freq - floor(vor_freq*10)/10) == 0.00 ) // X Mode
        {
            d_Mode = XMode;
            d_Mode_Delay = XMODE_DELAY_RX ;
            cmd = MODEX_CMD;
        }else if ( abs(vor_freq - floor(vor_freq*10)/10 - 0.05) < 1e-5) // Y Mode
        {
            d_Mode = YMode;
            d_Mode_Delay = YMODE_DELAY_RX;
            cmd = MODEY_CMD;
        }
        d_Detection_Delay = (int) round(HALF_PULSE_DELTA/MHZ*d_samp_rate);
        d_first_negative_index = (int) round(d_Mode_Delay/MHZ*d_samp_rate);
        d_first_positive_index =d_first_negative_index + 1;
        message_port_pub(
                    pmt::mp("to_tx"),
                    pmt::from_long(cmd));

    }
    //Message handlers
    void
    dme_rx_impl::cmd_handler(pmt::pmt_t msg)
    {
        if(pmt::is_dict(msg))
        {
            std::string cmd_lab = pmt::symbol_to_string((pmt::nth(1,msg)));
            std::cout<<d_vor_lab<<" == "<<cmd_lab<<" = "<<(cmd_lab==d_vor_lab) <<std::endl;
            if(cmd_lab==d_vor_lab)
            {
                float vor_freq_cmd=pmt::to_float(pmt::nth(2,msg));
                set_mode(vor_freq_cmd);
            }
            std::cout<<"[test]"<<(cmd_lab == "BIAS")<<std::endl;
            if(cmd_lab == "BIAS")
            {
                d_bias = pmt::to_long(pmt::nth(2,msg));
                std::cout<<"[d_bias]"<<d_bias<<std::endl;
            }
        }
    }

      void
      dme_rx_impl::tx_handler(pmt::pmt_t msg)
      {
          /*std::cout<<"[debug] size Rx/TX "<<
                     d_Received_Jitters_v.size()<<"/"<<
                     d_Transmitted_Jitters_v.size()<<std::endl;
                     */
          d_counter_tx_pulses++;
          long int sequence_no = pmt::to_uint64(msg);
          if (d_Transmitted_Jitters_v.size() < 80)
          {
              d_Transmitted_Jitters_v.push_back( sequence_no );
              //cout << "\t Message " <<  d_Transmitted_Jitters_v.size() << endl;
          } else
          {
              d_Transmitted_Jitters_v.push_back(   sequence_no  );
              d_Transmitted_Jitters_v.erase(d_Transmitted_Jitters_v.begin());
              d_counter_for_print_warning++;
              if (d_counter_for_print_warning == 120 )
              {
                  d_counter_for_print_warning =0;
                  std::cout << "\n****************************************************\n" <<
                               "DME:  Any pulses replay received in a long time \n" <<
                               "****************************************************\n";
              }
          }
          if ( d_Transmitted_Jitters_v.size() > 0)
          {
              while ( d_Received_Jitters_v.size() > 0 )
              {
                /*
                  std::cout<<std::dec<<"[debug] jitters==>\t"<<d_Received_Jitters_v[0]<<"\t"<<d_Transmitted_Jitters_v[0]<<
                  "\t diff "<<d_Received_Jitters_v[0]-d_Transmitted_Jitters_v[0]<<"\n";
                */
                  if ( d_Received_Jitters_v[0] < d_Transmitted_Jitters_v[0] + 50  )
                  {
                      //Received jitters TOO OLD and outside of the window. They are discarded
                      if(true)
                      {
                          /*cout << "Discarded Rx Jitter\t\t"  <<  d_Received_Jitters_v.size() <<
                                  " " <<  d_Received_Jitters_v.size()  << "\tRoJ " << d_Received_Jitters_v[0] << "  ToJ " << Transmitted_Jitters[0] <<
                                  "\tDifference" << dec << d_Received_Jitters_v[0] -d_Transmitted_Jitters_v[0]  <<endl;
                              */
                      }
                      d_Received_Jitters_v.erase(d_Received_Jitters_v.begin());
                  }else if(d_Received_Jitters_v[0] - d_Transmitted_Jitters_v[0]  < d_NSamples_WINDOW_LENGTH)
                  {
                      // RECEIVED JITTERS INSIDE THE WINDOW -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
                      d_nsamples_diff= d_Received_Jitters_v[0] - d_Transmitted_Jitters_v[0];
                      d_pairs_txseq_and_difference_v.push_back( std::make_pair(d_counter_tx_pulses,  d_nsamples_diff ) )    ;
                      if(d_pairs_number_and_ocurrence_v.empty() )
                      {
                          d_pairs_number_and_ocurrence_v.push_back(
                                      std::make_pair(1,  d_nsamples_diff) )    ;
//                          std::cout << "Occurrences is empty " << endl;
//                                  cout << "FIRST ocurrence \t" << d_pairs_number_and_ocurrence_v[0].first << " " <<
//                                  d_pairs_number_and_ocurrence_v[0].second <<
//                                  "\t\tDifferent number of occurencies" <<  d_pairs_number_and_ocurrence_v.size() <<"\n";
                      }else
                      {
                          bool differencem_is_in_ocurrences = false;
                          int size_pairs_nu_and_ocur = d_pairs_number_and_ocurrence_v.size();
                          for (int m = 0; m < size_pairs_nu_and_ocur; m++)
                          {
//                              std::cout << "pairs[m]" << d_pairs_number_and_ocurrence_v[m].second << "\t\tnsamples_diff" <<  d_nsamples_diff  << std::endl;
                              if (abs(d_pairs_number_and_ocurrence_v[m].second -  d_nsamples_diff  )< TIME_TOL)
                              {
                                  d_pairs_number_and_ocurrence_v[m].first++;
                                  d_pairs_number_and_ocurrence_v[m].second = d_nsamples_diff;
                                  d_last_tx_Jitter_rx_ok = d_Transmitted_Jitters_v[0];
                                  //	cout << "ocurrence \t" << pairs_number_and_ocurrence[m].first << " " <<
                                  //	pairs_number_and_ocurrence[m].second <<
                                  //	"\t\tDifferent number of occurencies" <<  pairs_number_and_ocurrence.size() <<"\n";
                                  //	differencem_is_in_ocurrences =true;
                                  break;
                              }
                          }
                          if(!differencem_is_in_ocurrences)
                          {
                              d_pairs_number_and_ocurrence_v.push_back( std::make_pair(1,  d_nsamples_diff) )    ;
                              //cout << "NEW OCURRENCE" << endl;
                          }
                      }
                      //d_Transmitted_Jitters_v.erase(d_Transmitted_Jitters_v.begin());
                      boost::mutex::scoped_lock lock(mutex);
                      d_Received_Jitters_v.erase(d_Received_Jitters_v.begin());
                      d_counter_recovered++;
                  }else if(d_Received_Jitters_v[0] - d_Transmitted_Jitters_v[0]  > d_NSamples_WINDOW_LENGTH)
                  {
                      d_Transmitted_Jitters_v.erase(d_Transmitted_Jitters_v.begin());
                      //The TX Pulse is discarded and not replays were received
                      if (d_In_Window_Jitters_v.size() > 0)
                      {
                      }
                  }
              }// end while
          }//if ( d_Transmitted_Jitters_v.size() > 0)
          if (!d_pairs_txseq_and_difference_v.empty())//ANALISYS OF THE RECECEIVED PULSES IN WINDOW
          {
              if (d_pairs_txseq_and_difference_v[ d_pairs_txseq_and_difference_v.size() -1 ].first - d_pairs_txseq_and_difference_v[0].first  > MAX_PULSES)
              {
                  int jiitters_to_delete = d_pairs_txseq_and_difference_v[0].first;
                  while (d_pairs_txseq_and_difference_v[0].first == jiitters_to_delete )
                  {
                      int size_pairs_nu_and_ocur = d_pairs_number_and_ocurrence_v.size();
                      for (int m = 0; m < size_pairs_nu_and_ocur; m++)
                      {
                          //std::cout << "pairs[m]" << d_pairs_number_and_ocurrence_v[m].second << "\t\tnsamples_diff" <<  d_nsamples_diff  << std::endl;
                          if (abs(d_pairs_number_and_ocurrence_v[m].second -  d_pairs_txseq_and_difference_v[0].second )< TIME_TOL)
                          {
                              if (d_pairs_number_and_ocurrence_v[m].first > 0)
                              {
                                  d_pairs_number_and_ocurrence_v[m].first--;
                              }
                              /*cout << "BORRAR\t" << pairs_number_and_ocurrence[m].first << " " <<
                                      pairs_number_and_ocurrence[m].second <<
                                      "\t\tDifferent number of occurencies" <<  pairs_number_and_ocurrence.size() <<"\n";
                                      */
                              break;
                          }
                      }
                      d_pairs_txseq_and_difference_v.erase(d_pairs_txseq_and_difference_v.begin());
                  }
                  d_greater_occurrence_pr.first  = 0;
                  d_greater_occurrence_pr.second = 0;
                  int size_pairs_nu_and_ocur = d_pairs_number_and_ocurrence_v.size();
                  for (int m = 0; m < size_pairs_nu_and_ocur; m++)
                  {
                      if( d_pairs_number_and_ocurrence_v[m].first > d_greater_occurrence_pr.first)
                          d_greater_occurrence_pr = d_pairs_number_and_ocurrence_v[m];
                  }
                  //cout<<"greater_occurrence.first "<<greater_occurrence.first<<"\n";
                  if (d_greater_occurrence_pr.first > MIN_GOT_PULSES)
                  {
//                      std::cout << "\n\n\n\the pairs are chedcked well" <<  "\n"<<
//                              "THE GREARET OCCURRENCE IS: " << d_greater_occurrence_pr.second <<
//                              "\t\tWITH no APPEARANCES "<< d_greater_occurrence_pr.first<<  "\n" <<
//                              "---------------------------------------------------"<<  "\n";

                      /*
                      time_t ctt = time(0);
                      //cout << asctime(localtime(&ctt)) << endl;
                      int timo = time(0);
                      /*
                      cout << "Time"<<  ctt   << "\tSequence Number  " << last_tx_Jitter_rx_ok << "\tTime Delay (samples) " <<
                       greater_occurrence.second << "\t Distance (NM)= " << (greater_occurrence.second - 58.6)*MUS2NMI << endl;
                      */
                      /*
                      pmt::pmt_t pmt_dme_log =  pmt::from_bool( goto_Tracking_Mode );
                      pmt::pmt_t log_message = pmt::make_dict();
                      */
                      //Computed result - TODO: Magic is an abomination -
                      /*
                      if(Mode == XMode){ pmt_Mode        = pmt::intern("x"); }else{
                          pmt_Mode        = pmt::intern("y");			}
                      if(Op_Mode == Search_Mode){ pmt_Mode        = pmt::intern("search"); }else{
                          pmt_Mode        = pmt::intern("track");			}


                      pmt_timestamp =  pmt::from_double(   get_time_double_ms()    );

                      //pmt_seq_key = pmt::intern("seq_number");
                      pmt_seq  = pmt::from_long(   last_tx_Jitter_rx_ok    );



                      pmt_delay       = pmt::from_double(greater_occurrence.second);
                      pmt_distance    = pmt::from_double(distance);

                      log_message = pmt::dict_add(log_message, pmt_distance_key, pmt_distance);
                      log_message = pmt::dict_add(log_message, pmt_delay_key, pmt_delay);
                      log_message = pmt::dict_add(log_message, pmt_Op_Mode_key, pmt_Op_Mode);
                      log_message = pmt::dict_add(log_message, pmt_Mode_key, pmt_Mode);

                      log_message = pmt::dict_add(log_message, pmt_seq_key, pmt_seq);
                      log_message = pmt::dict_add(log_message, pmt_timestamp_key, pmt_timestamp);

                      log_message = pmt::dict_add(log_message, pmt_ID_key, pmt_label);

                      //message_port_pub(pmt::mp("dme_log"), log_message);

                        */

                      //Computed result - TODO: Magic is an abomination -
                      //double distance = (d_greater_occurrence_pr.second - 50)*MUS2NMI;
                      //double distance = (d_greater_occurrence_pr.second - 46);//RADAR_NM_US;
                      //TODO:decument delay 50
                      //double distance = (d_greater_occurrence_pr.second-50+53)*MUS2NMI;

                      int main_delay = d_Mode==XMode ? MAIN_DELAY_X : MAIN_DELAY_X ;
                      //NOTE: adding half a sample (0.5) improves the precision
                      double distance = (d_greater_occurrence_pr.second+0.5-main_delay)*MUS2NMI;
                      // result to msg
                      message_port_pub(pmt::mp("dist"), pmt::list3(
                                           pmt::intern(d_dme_id),
                                           pmt::intern("DME_dist"),
                                           pmt::from_double(distance)
                                           ));

                      if( d_Op_Mode == Search_Mode )
                      {
                          std::cout << "WE GO TO Tracking_Mode" << std::endl;
                          d_Op_Mode = Tracking_Mode;
                          message_port_pub(
                                      pmt::mp("to_tx"),
                                      pmt::from_long(TRACK_CMD));
                      }
                  }else if( d_Op_Mode == Tracking_Mode )
                  {
                      std::cout << "WARNING!: WE Return to SEARCH MODE" << std::endl;
                      d_Op_Mode = Search_Mode;
                      message_port_pub(
                                  pmt::mp("to_tx"),
                                  pmt::from_long(SEARCH_CMD));
                  }
              } //if (pairs_txseq_and_difference[ pairs_txseq_and_difference.size() -1 ].first - pairs_txseq_and_difference[0].first  > MAX_PULSES)
          } //if (!pairs_txseq_and_difference.empty())
      }//dme_rx_impl::tx_handler(pmt::pmt_t msg)

    /***************************************************
     * Work!!!
     * ***********************************************/

    int
    dme_rx_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        const float *in = (const float *) input_items[0];
        const uint64_t *fpga_time_in = (const uint64_t *) input_items[1];
        float *out = (float *) output_items[0];
        //Increasing the sample-zero pointer to include the history;
        in+=(d_history_samples -1 -d_first_positive_index);

        d_ndetections = 0;


        int nsamples_to_process = noutput_items + d_first_positive_index;

        volk_32f_x2_multiply_32f(
                    d_in_times_four_p,
                    in - d_Detection_Delay,
                    d_constant_4_vector_p ,
                    nsamples_to_process  );

        volk_32f_x2_subtract_32f(
                    d_Zero_Cross_p,
                    in,
                    d_in_times_four_p  ,
                    nsamples_to_process );
        volk_32f_x2_subtract_32f(
                    d_Detection_in_threshold_float_p,
                    in,
                    d_Threshold_vector_p, nsamples_to_process  );

        for (int i =0; i <  nsamples_to_process ; i++)
        {
            if (d_Zero_Cross_p[i] > 0){
                d_Detection_p[i]  = 0xFF; d_Not_Detection_p[i] = 0x00;//NOTE:FF/00?
            }else
            {
                d_Detection_p[i]  = 0x00; d_Not_Detection_p[i] = 0xFF;
            }
            if (d_Detection_in_threshold_float_p[i]>0){
                d_Detection_in_threshold_p[i] = 0xFF;
            }else
            {
                d_Detection_in_threshold_p[i] = 0x00;
            }
        }

        int nop_bool = int( ceil(nsamples_to_process/4)  );

        volk_32i_x2_and_32i(
                    (int32_t *)d_Detection_p   ,
                    (int32_t *)d_Detection_in_threshold_p,
                    (int32_t *)d_Detection_p,
                    nop_bool  );
        volk_32i_x2_and_32i(
                    (int32_t *)d_Receive_Jitter_p,
                    (int32_t *)(d_Detection_p ) ,
                    (int32_t *)(d_Not_Detection_p+1),
                    nop_bool   );
        volk_32i_x2_and_32i(
                    (int32_t *)d_Receive_Jitter_p,
                    (int32_t *)d_Receive_Jitter_p,
                    (int32_t *)(d_Detection_p +  d_first_negative_index  ),
                    nop_bool   );
        volk_32i_x2_and_32i(
                    (int32_t *)d_Receive_Jitter_p, (int32_t *)d_Receive_Jitter_p,
                    (int32_t *)(d_Not_Detection_p +  d_first_positive_index),
                    nop_bool   );

        for (int i =0; i <  noutput_items ; i++)
        {
            if (d_Detection_p[i])
            {
                if (d_ndetections==0)
                {
                    d_ndetections++;

                    //cout << " noutputs " << noutput_items << " ";
                }
                //cout<<" x"<<nitems_read(0)+i<< " i"<< i;
            }
            if (d_Receive_Jitter_p[i] )
            {
                if (   d_Received_Jitters_v.size() < 20000 && d_Received_Jitters_v.size() % 2 ==0)
                {

                    boost::mutex::scoped_lock lock(mutex);
                    d_Received_Jitters_v.push_back(fpga_time_in[i]+d_bias+LAB_OFFSET);
//                    std::cout << "  Received exact: " << fpga_time_in[i]<<  std::endl;

                    d_counter_rx_pulses++;
                }
                /*
               cout << " Receive_Jitter " << std::setprecision(14) <<   Received_Jitters[ Received_Jitters.size() - 1 ]  <<
               "\t\tinteger " << nitems_read(0) + i  -d_calibration_value <<
                 "\ttx_pulses "    <<    counter_tx_pulses   <<  "\t Rx_pulses " <<
               counter_rx_pulses <<  "\t PPRF= " <<  counter_rx_pulses*1.0e6/(nitems_read(0)) <<
               "\t\tinterpolation= " <<   (Zero_Cross[i]/(Zero_Cross[i] - Zero_Cross[i+1] ))  <<
                endl;
                */

            }
        }
        memcpy( out ,  d_Zero_Cross_p, noutput_items*4 );
        return noutput_items;
    }

  } /* namespace dme */
} /* namespace gr */

