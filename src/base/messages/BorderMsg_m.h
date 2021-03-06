//
// Generated file, do not edit! Created by nedtool 4.6 from base/messages/BorderMsg.msg.
//

#ifndef _BORDERMSG_M_H_
#define _BORDERMSG_M_H_

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0406
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include "Coord.h"
// }}

/**
 * Class generated from <tt>base/messages/BorderMsg.msg:34</tt> by nedtool.
 * <pre>
 * // used as a self packet containing the position, direction and BorderPolicy
 * //
 * // @author Daniel Willkomm
 * //
 * packet BorderMsg
 * {
 *     int policy;    // border policy to use for the host
 *     Coord startPos; // new start position of the host
 *     Coord direction; // new direction of the host
 * }
 * </pre>
 */
class BorderMsg : public ::cPacket
{
  protected:
    int policy_var;
    Coord startPos_var;
    Coord direction_var;

  private:
    void copy(const BorderMsg& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const BorderMsg&);

  public:
    BorderMsg(const char *name=NULL, int kind=0);
    BorderMsg(const BorderMsg& other);
    virtual ~BorderMsg();
    BorderMsg& operator=(const BorderMsg& other);
    virtual BorderMsg *dup() const {return new BorderMsg(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual int getPolicy() const;
    virtual void setPolicy(int policy);
    virtual Coord& getStartPos();
    virtual const Coord& getStartPos() const {return const_cast<BorderMsg*>(this)->getStartPos();}
    virtual void setStartPos(const Coord& startPos);
    virtual Coord& getDirection();
    virtual const Coord& getDirection() const {return const_cast<BorderMsg*>(this)->getDirection();}
    virtual void setDirection(const Coord& direction);
};

inline void doPacking(cCommBuffer *b, BorderMsg& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, BorderMsg& obj) {obj.parsimUnpack(b);}


#endif // ifndef _BORDERMSG_M_H_

