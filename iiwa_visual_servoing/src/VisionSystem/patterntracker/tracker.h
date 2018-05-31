/*
    Tracker base class

    2015 Lin Zhang, Menglong Ye
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef TRACKER_H
#define TRACKER_H


/*
Class that defines what is a feature generic tracker.

A tracker is able to track features with parameters expressed in:

- in the camera frame cP. These parameters are located in the public attribute dvrkTracker::cP.
- in the image plane p. These parameters are located in the public attribute dvrkTracker::p. They correspond to normalized coordinates of the feature expressed in meters.
*/

namespace dvrk
{
class Tracker
{

public:
    bool cPAvailable;

public:
    //! Default initialization.
    void init();
    //! Default constructor.
    Tracker();
    //! Copy constructor.
    Tracker(const Tracker &tracker);
    //! Copy operator.
    Tracker &operator=(const Tracker &tracker);

    //! Destructor.
    virtual ~Tracker() {}
} ;

}   // namespace dvrk


#endif  // TRACKER_H
