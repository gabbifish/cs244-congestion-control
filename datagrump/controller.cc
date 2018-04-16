#include <iostream>
// #include <math.h>
#include <cmath> 

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* AIMD constants */
const unsigned int ADD_INCREASE = 1;
const float MULT_DECREASE = .5;

/* Initial window size is 10. */
float the_window_size = 10; 

/* CONSTANTS for QUIC using time based loss detection. */
/* Maximum number of tail loss probes before an RTO fires. */
const uint32_t kMaxTLPs = 2;  

/* Maximum reordering in packet
   number space before FACK style loss detection considers a packet
   lost. */
const uint32_t kReorderingThreshold = 3; 

/* Maximum reordering in time
   space before time based loss detection considers a packet lost.
   In fraction of an RTT. */
float kTimeReorderingFraction = 0.125;

/* Minimum time in the future a tail loss probe alarm may be set for. Defualt 10ms. */
const uint32_t kMinTLPTimeout = 10;  

/* Minimum time in the future an RTO alarm may be set for. Defualt 200ms. */
const uint32_t kMinRTOTimeout = 200;

/* The length of the peer's delayed ack timer. Default 25 ms. */
const uint32_t kDelayedAckTimeout = 25; 

/* The default RTT used before an RTT sample is taken. Default 100ms. */
const uint32_t kDefaultInitialRtt = 100;

/* UPDATED VARS */
uint32_t tlp_count = 0;
uint32_t rto_count = 0;
uint32_t reordering_threshold = 1000000; /* Approximating inf. */
uint32_t time_reordering_fraction = kTimeReorderingFraction;
uint32_t loss_time = 0;
float smoothed_rtt = 0.0;
float rttvar = 0.0;
float min_rtt = 0.0;
uint32_t max_ack_delay = 0;
uint32_t largest_sent_before_rto = 0;
uint32_t time_of_last_sent_packet = 0;
uint64_t largest_sent_packet = 0;
uint64_t largest_acked_packet = 0;
float latest_rtt = 0.0;

float ssthresh = 10000; /* Approximating inf. */
uint32_t packets_in_flight = 0;
uint32_t end_of_recovery = 0;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return (int)the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }

  packets_in_flight += 1;
  if (after_timeout) {
    ssthresh = the_window_size;
    the_window_size *= 0.5;
  }
}

void update_rtt (uint64_t latest_rtt) 
{
  if (smoothed_rtt == 0.0) {
    smoothed_rtt = latest_rtt;
    rttvar = latest_rtt / 2.0;
  }
  else {
    rttvar = 0.75 * rttvar + .25 * float(abs(smoothed_rtt - latest_rtt));
    smoothed_rtt = .875 * smoothed_rtt + .125 * latest_rtt;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{

  largest_acked_packet = sequence_number_acked;
  latest_rtt = float(timestamp_ack_received - send_timestamp_acked);
  update_rtt(latest_rtt);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)."
   << " RTT is " << latest_rtt << " while smoothed RTT is " << smoothed_rtt
	 << endl;
  }

  packets_in_flight -= 1;
  if (sequence_number_acked < end_of_recovery) {
    return;
  }
  if (latest_rtt < 1.40*smoothed_rtt) {
    if (the_window_size < ssthresh) {
      the_window_size += 1.0;
    }
    else {
      the_window_size += 1.0 / the_window_size;
    }
  }
  else {
    ssthresh = the_window_size;
    the_window_size *= 0.5;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 60;
}
