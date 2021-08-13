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

#ifndef INCLUDED_DME_DME_TIME_IMPL_H
#define INCLUDED_DME_DME_TIME_IMPL_H

#include <dme/dme_time.h>

#define DME1_MSK    0x80000000
#define DME2_MSK    0x40000000
#define TIME_MSK    0x3FFFFFFF
#define TIME_LAP    0x40000000
#define TIME_SCL    40 //Time scale
#define NORMAL_STEP 40

namespace gr {
namespace dme {


class dme_time_impl : public dme_time
{
private:
    uint64_t d_buffer; /// time stamp
    int d_old; /// old value
    int d_hold; /// hold  value
    int d_dme1_counter; /// dme1_pulse counter
    int d_dme2_counter; /// dme2_pulse counter

public:
    dme_time_impl();
    ~dme_time_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace dme
} // namespace gr

#endif /* INCLUDED_DME_DME_TIME_IMPL_H */

