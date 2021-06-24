//
// Angle Compensator for Nav Scanner
//

#ifndef SICK_SCAN_ANGLE_COMPENSATOR_H
#define SICK_SCAN_ANGLE_COMPENSATOR_H

#include <string>
#include <vector>
#include <assert.h>
class AngleCompensator
{
public:
  double compensateAngleInRadFromRos(double angleInRadFromRos);
  double compensateAngleInRad(double angleInRad);
  double compensateAngleInDeg(double angleInDeg);
  int parseAsciiReply(const char *asciiReply);
  int parseReply(bool isBinary, std::vector<unsigned char>& replyVec);
  std::string getHumanReadableFormula(void);
  void testbed();
  AngleCompensator()
  {
    assert(0); // forbidden!
  }
  AngleCompensator(bool _useNegSign)
  {
    useNegSign = _useNegSign;
  }
private:

  double amplCorr;
  double phaseCorrInDeg;
  double offsetCorrInDeg;
  double phaseCorrInRad;
  double offsetCorrInRad;
  bool useNegSign; // for NAV310

};


#endif //SICK_SCAN_ANGLE_COMPENSATOR_H
