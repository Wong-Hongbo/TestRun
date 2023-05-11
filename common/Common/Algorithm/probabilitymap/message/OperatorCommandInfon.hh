/*
OperatorCommandInfon.hh

This C++ header file defines the NML Messages for OperatorCommandInfon
Template Version 1.1

MODIFICATIONS:
Wed June 04 10:22:05 CST 2014	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef OPERATORCOMMANDN_HH
#define OPERATORCOMMANDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"
#include "usertype.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define OPERATORCOMMAND_MSG_TYPE 1666
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


//UGV_MODE_CMD ��UGV_ACTION_CMD��ÿ֡�м��ȡֵ���ڷ�Χ�ڣ���������֡������


/*local coordinate*/

class OPERATORCOMMAND_MSG : public NMLmsgEx
{
public:

	//Constructor
	OPERATORCOMMAND_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
	int MessageID;                      // ��ϢID
	// int MessageSeqNum;               // ��Ϣ���к�
	int TimeFlag;                       // ʱ�䣬��λms
	int DestinationID;		            // ��ַ�룬ָ�������豸 
	int Ugv_mode;	                    // ��һ��������������ͬ����ģʽ�������������������ӣ� ������  UGV_MODE_CMD
	int Ugv_action;	                    // �ڶ�����������ͬ����ģʽ�¶�Ӧ�Ķ��� ������ UGV_ACTION_CMD
	int CommandData[4];                 // �������ݣ�������
  LOCAL_COORDINATE position_frame;
};
// Declare NML format function
extern int OperatorCommandFormat(NMLTYPE, void *, CMS *);

#endif 	// OPERATORCOMMANDINFON_HH
