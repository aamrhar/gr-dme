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
 
#ifndef INCLUDED_DME_CONSTANTS_H
#define INCLUDED_DME_CONSTANTS_H

#define MHZ 1000000
#define PRECISION 4

typedef enum  { XMode, YMode } XYModetype;
typedef enum  {Search_Mode,Tracking_Mode } Op_Modetype;

// Pair pulse spacing (mus) X-Mode (reception)
#define XMODE_DELAY_RX 12

// Pair pulse spacing (mus) Y-Mode (reception)
#define YMODE_DELAY_RX 36

// Wafeform Guard (mus) after Pair of Pulses Generation
#define MODE_DELAY_GUARD 6

// DME pulse Gaussian amplitude variance (MHz²)
#define PULSE_VAR 0.45

// Time spacing between TOA and maximum amplitude (mus). TOA defined as Amp = Max/2
#define HALF_PULSE_DELTA 1.75

// Ground station delay (mus)
// #define GS_DELAY 50
#define GS_DELAY 44

// Ground station delay (mus) //NOTE: this must be a radio mile?
#define MUS2NMI 0.08097165991902834 //0.08091252699784017

// Radar nautical mille in us
#define RADAR_NM_US 12.35

/*
 * Implementation parameters
 */
// Time during which the receiver allows pairs of pulses after transmitted ones (mus)
#define WINDOW_LENGTH 2530

// Pair of pulses per second -- Search mode
#define PPS_SEARCH 150

// Pair of pulses per second -- Track mode
#define PPS_TRACK 16

// Number of missed pulses to go back to Search Mode
#define MAX_MISSED_PULSES 3
// #define MAX_MISSED_PULSES 4

#define MIN_GOT_PULSES 4
// #define MAX_MISSED_PULSES 4

// Number of pulses in a set (one set per measurement)
#define MAX_PULSES 6
// #define MAX_PULSES 8

// The maximum delay allowed in TX-to-RX path (mus)
// #define MAX_DELAY 100000000
#define MAX_DELAY 300000

#define BIAS_INI 0
// The bias increment during calibration mode (in % of WINDOW_LENGTH)
// Because: "Predicted_Time = Time - Bias_Inc;" This cannot exceed 50%
// #define BIAS_INC 50
 #define BIAS_INC 50

// The coefficient of the first order IIR filter to smooth time measurements while tracking
#define ALPHA 0.75
 // #define ALPHA 1

// The tolerance for losing track (mus) (range gate)
/* (250 knots / 3600 (seg/hr) * (1/16)*1.5              * 3                   + 0.04 nmi)        / MUS2NMI
    Max Speed                   [Track_Min+Track_Range]   [MAX_MISSED_PULSES]   [range accuracy]           */
// #define TRACK_TOL 0.8
#define TRACK_TOL 1

// The tolerance to determine a different delay measurement (mus)
#define TIME_TOL 0.5
// #define TIME_TOL 0.25

// Maximum number of measurements to increase the threshold (more than)
// #define MAX_MEASUREMENTS 10
 #define MAX_MEASUREMENTS 10

// Minimum number of measurements to decrease the threshold (less than)
// #define MIN_MEASUREMENTS 1
#define MIN_MEASUREMENTS 5

// Minimum detection threshold (actual threshold is 2x this value)
#define MIN_THRESHOLD 0.00022 // 0.05

// Minimum detection threshold (actual threshold is 2x this value)
#define MAX_THRESHOLD 350000000 // (1 / 4) So it never is > 1.0

// Simulation offset (found empirically)
#define SIM_OFFSET 53
// lab offset (found empirically)
#define LAB_OFFSET 44
// Ground satation delay for XMODE in samples
#define MAIN_DELAY_X 50

// Ground satation delay for YMODE in samples
#define MAIN_DELAY_Y 56

// RX-->TX commands

#define MODEX_CMD 0 //set mode to X
#define MODEY_CMD 1 //set mode to Y
#define SEARCH_CMD 2 //set op_mode to search
#define TRACK_CMD 3 //set op_mode to track

#endif //#ifndef INCLUDED_DME_CONSTANTS_H
