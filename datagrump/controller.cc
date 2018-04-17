#include <iostream>
// #include <math.h>
#include <cmath> 

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Initial window size is 10. */
float prev_cwnd = 0.0; 
float cwnd = 1.0; 
float min_cwnd = 4.0;

float smoothed_rtt = 0.0;
float rttvar = 0.0;
uint64_t min_rtt = 0;
uint64_t min_rtt_update = 0;
uint64_t standing_rtt = 0;
uint64_t standing_rtt_update = 0;
uint64_t min_rtt_last_update = 0;

float delta = 0.2;
float v = 1.0; /* Velocity param */
bool direction_up = 1;
uint32_t rtts_up = 0;

bool slow_start = true;
/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << cwnd << endl;
  }

  return (int)(cwnd+0.5);
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
  prev_cwnd = int(cwnd+0.5);
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
  if ( debug_ ) {
   cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)."
	 << endl;
  }

  /* Update smoothed rtt and rtt variance */
  uint64_t latest_rtt = timestamp_ack_received - send_timestamp_acked;
  if (smoothed_rtt == 0.0) {
    smoothed_rtt = latest_rtt;
    rttvar = latest_rtt / 2.0;
  }
  else {
    rttvar = 0.75 * rttvar + .25 * float(abs(smoothed_rtt - latest_rtt));
    smoothed_rtt = .875 * smoothed_rtt + .125 * latest_rtt;
  }

  if (latest_rtt < min_rtt || min_rtt == 0 || timestamp_ack_received >= min_rtt_update + 10000) {
    min_rtt = latest_rtt;
    min_rtt_update = timestamp_ack_received;
  }

  float queue_delay = smoothed_rtt - min_rtt;
  cerr << "min rtt is " << min_rtt << " latest rtt is " << latest_rtt << endl;
  cerr << "queueing delay " << queue_delay << endl;
  float lambda_est = 1/(delta * queue_delay);
  float lambda = cwnd/smoothed_rtt;

  cerr << "lambda is " << lambda << " and lambda_est is " << lambda_est << endl;
  if (lambda <= lambda_est) {
    if (slow_start) {
      cwnd *= 2;
    }
    else {
      cerr << "v/(delta*cwnd) is " << v/(delta*cwnd) << endl;
      cwnd += v/(delta*cwnd);
    }
  }
  else {
    if (slow_start) {
      slow_start = false;
    }
    cwnd -= v/(delta*cwnd);
  }

  /* Update v */
  bool prev_direction_up = direction_up;
  if (int(cwnd+0.5) > prev_cwnd) {
    direction_up = true;
    rtts_up++;
  } 
  else if (int(cwnd+0.5) <= prev_cwnd) {
    direction_up = false;
    rtts_up = 0;
  }

  if (direction_up && prev_direction_up == direction_up && rtts_up >= 3) {
    v *= 2;
  }
  else if (!direction_up) {
    v = 1;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 50;
}
