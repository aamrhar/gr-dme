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


#ifndef INCLUDED_DME_DME_TIME_H
#define INCLUDED_DME_DME_TIME_H

#include <dme/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace dme {

    /*!
     * \brief This block allows to synch system to FPGA's
     * clk
     * input1 : FPGA time stamp (int32)
     * format b31=> DME1 tx detected; b30 DME2 tx detected;
     * b29-0: timestamp at 40Mhz
     * output1 : accumulatetd time stamp (u_int64)
     * output 2: accumulated time_stamps if dme1_tx detected (pmt-unint64)
     * output 2: accumulated time_stamps if envent2 (pmt-unint64)
     * \ingroup dme
     *
     */
    class DME_API dme_time : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<dme_time> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dme::dme_time.
       *
       * To avoid accidental use of raw pointers, dme::dme_time's
       * constructor is in a private implementation
       * class. dme::dme_time::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_DME_TIME_H */

