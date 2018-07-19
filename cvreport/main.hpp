#ifndef CVREPORT_MAIN_H_
#define CVREPORT_MAIN_H_

#if defined(__linux__) || defined(__APPLE__)

#define kOPENCV_KEY_ENTER 10

#elif _MSC_VER

#define kOPENCV_KEY_ENTER 13

#endif

#define kOPENCV_KEY_ESCAPE 27

class SubProcedure
{
public:
	virtual void Run() = 0;
private:
};

#endif  // CVREPORT_MAIN_H_
