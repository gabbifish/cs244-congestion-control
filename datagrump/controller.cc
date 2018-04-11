#include <iostream>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* AIMD constants */
const unsigned int ADD_INCREASE = 1;
const float MULT_DECREASE = .5;

const float MAX_WINDOW_SZ = 200;
float the_window_size = 1;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{
  unsigned int effective_window_size = floor(min(the_window_size,MAX_WINDOW_SZ));

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size
   << ", effective window size is " << effective_window_size
   << endl;
  }

  return effective_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{
  /* AIMD: decrease window size if datagram was sent due to timeout */
  if (after_timeout && the_window_size > 1) {
    the_window_size *= MULT_DECREASE;
  }

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
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
  /* AIMD: increase window size when ack received
     (duplicate acks are not possible since sender never
     sends same sequence number twice and
     receiver only acks received packets once)*/
  if (the_window_size < MAX_WINDOW_SZ) {
    the_window_size += ADD_INCREASE / the_window_size;
  }

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 25;
}
