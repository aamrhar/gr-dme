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


#ifndef INCLUDED_DME_DME_TX_H
#define INCLUDED_DME_DME_TX_H

#include <dme/api.h>
#include <gnuradio/sync_block.h>
#include "dme_constants.h"

namespace gr {
  namespace dme {

    /*!
     * \brief   This block generate DME signal
     *          it's also DME RX block slave
     * \ingroup dme
     *
     */
    class DME_API dme_tx : virtual public gr::sync_block
    {
     public:
        /*****************************************
         * Callbacks
         * ****************************************/
        virtual void Set_Mode(float vor_freq)=0;

      typedef boost::shared_ptr<dme_tx> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of dme::dme_tx.
       *
       * To avoid accidental use of raw pointers, dme::dme_tx's
       * constructor is in a private implementation
       * class. dme::dme_tx::make is the public interface for
       * creating new instances.
       */
      static sptr make(float samp_rate, float vor_freq, int id_num);
    };

  } // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_DME_TX_H */

