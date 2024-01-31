#ifndef __ADAS__
#define __ADAS__

#include <sstream>  // for parsing ROI
#include <thread>

#include <cstring>
#include <memory>
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include <deque>
#include <thread>
#include <condition_variable>

#ifdef SAV837
// SGS
#include "mi_vpe_datatype.h"
#include "mi_scl_datatype.h"
#include "dla_base.h"
#include "dla_base_class.h"
#endif

// WNC
#include "dataStructures.h"
#include "dla_config.hpp"
#include "adas_config_reader.hpp"
#include "yolo_adas.hpp"
#include "yolo_adas_postproc.hpp"
#include "matching2D.hpp"
#include "lane_finder.hpp"
#include "lane_line_det.hpp"
#include "optical_flow.hpp"
#include "object_tracker.hpp"
#include "json_log.hpp"
#ifdef QCS6490
#include "ldw.hpp"
#include "fcw.hpp"
#endif
#ifdef SAV837
#include "wnc_ldw.hpp"
#include "wnc_fcw.hpp"
#endif
#include "logger.hpp"

using namespace std;

#define ADAS_VERSION "0.4.1"
#define ADAS_SUCCESS 1
#define ADAS_FAILURE 0

enum ADAS_EVENTS
{
  ADAS_EVENT_NORMAL,
  ADAS_EVENT_LDW_FCW,
  ADAS_EVENT_LDW,
  ADAS_EVENT_FCW
};


struct ADAS_DRAW_RESULTS
{
	int yVanish;
  	ROI vehicleROI;
	ROI riderROI;
	ROI humanROI;
	bool isLaneDeparture;
	bool isForwardCollision;
	bool isDraw;
	std::vector<BoundingBox> humanBBoxList;
	std::vector<BoundingBox> riderBBoxList;
	std::vector<BoundingBox> vehicleBBoxList;
	std::vector<BoundingBox> roadSignBBoxList;
	std::vector<BoundingBox> stopSignBBoxList;
	std::vector<Object> trackedObjList;
	cv::Mat matCalibMask;
	cv::Mat matLaneMask;
	cv::Mat matMergeLineMask;
	cv::Mat matColorLaneMask;
	cv::Mat matColorLineMask;
	cv::Mat matResult;
};


class ADAS
{
  	public:
		ADAS(std::string configPath);
		~ADAS();

		// === Frame === //
		int m_frameIdx = 0;
		int m_frameStep = 4;

		// === Display === //
		bool m_dsp_results = false;
		bool m_dbg_saveLogs = false;
		bool m_dbg_saveImages = false;
		bool m_dbg_saveRawImages = false;

		// === Other === //
		bool m_estimateTime = false;

		// === Main Processing === //
		#ifdef SAV837
		void IpuShowOutput(ADAS_Results &result);
		void IpuRunProcess();
		bool _scaleToModelSize(MI_SYS_BufInfo_t* pstBufInfo);
		#endif
		#ifdef QCS6490
		bool run(cv::Mat &imgFrame);
		bool run(cv::Mat &imgFrame, cv::Mat &resultMat);
		#endif

		// === Thread Management === //
		void stopThread();

		// === Work Flow === //
		bool _modelInfernece();
		bool _laneLineDetection();
		bool _objectDetection();
		bool _objectTracking();
		bool _laneDepartureDetection();
		bool _forwardCollisionDetection();
		
		// === Output === //
		int getDetectEvents();
		void getCrossWalkLine();
		float getFollowingDistance();
		void getResults(ADAS_Results &result);
		void getResultImage(cv::Mat &imgResult);

		// === Utils === //
		void _updateFrameIndex();

		// === Results === //
		void _saveRawImages();
		void _showDetectionResults();
		void _saveDetectionResults();
		void _saveDrawResults();
		void _showADASResult();

		// === Debug === //
		void _drawResults();
		void _drawLDWROI();
				
		// === YOLO-ADAS === //
		OpticalFlow* m_opticalFlow;
		YOLOADAS_POSTPROC* m_yoloADAS_PostProc;
		YOLOADAS* m_yoloADAS;
		cv::Mat m_img;

		// === Post Processing Result (for Multi-Thread) === //
		POST_PROC_RESULTS m_procResult;

	protected:

		#ifdef SAV837
		typedef struct
		{
			ST_Rect_T rect;
			char szObjName[64];
		}ST_DlaRectInfo_T;

		typedef struct _Point_t
		{
			MI_S32 x;
			MI_S32 y;
		} Point_t;

		typedef struct
		{
			Point_t leftStart;
			Point_t leftEnd;
			Point_t rightStart;
			Point_t rightEnd;
		} ST_DlaLineInfo_T;

		typedef struct
		{
			MI_S32 y;
		} ST_DlaVanishInfo_T;


		// === IPU related === //
		MI_S32 IpuInit();
		MI_S32 IpuDeInit();
		MI_S32 IpuGetSclOutputPortParam(void);
		// void IpuShowOutput(wnc_ADAS_Results &result);
		MI_S32 IpuGetRectInfo(ST_DlaRectInfo_T* pstRectInfo, wnc_ADAS_Results &result);
		MI_S32 IpuGetLineInfo(ST_DlaLineInfo_T* pstLineInfo, wnc_ADAS_Results &result);
		MI_S32 IpuGetVanishInfo(ST_DlaVanishInfo_T* pstVanishInfo, wnc_ADAS_Results &result);
	
		// === Configuration === //
		MI_S32 IpuReadConfig(string configPath);
		#endif

	private:

		// === Initilization === //
		bool _init(std::string configPath);
		bool _initROI();
		bool _readDebugConfig();
		bool _readDisplayConfig();
		bool _readShowProcTimeConfig();
    	void _saveDetectionResult(std::vector<std::string>& logs);

		// === Thread Management === //
		bool _runShowLogsFunc();
		bool _runDrawResultFunc();

		// === Debug === //
		void _drawLaneDetector();
		void _drawBoundingBoxes();
		void _drawLaneLineBoundingBoxes();
		void _drawTrackedObjects();
		void _drawLaneLines();
		void _drawInformation();


		// === Config === //
		ADAS_ConfigReader* m_adasConfigReader;
		ADAS_Config_S* m_config;

		// === Input Size === //
		int m_videoWidth = 576;
		int m_videoHeight = 320;
		int m_modelWidth = 576;
		int m_modelHeight = 320;

		// === Mask Size === //
		int m_maskWidth = 0;
		int m_segWidth = 0;
	
		#ifdef SAV837
		// === SGS SGL Related === //
									// SCL0: For Video Encoder
		MI_S32 m_s32StreamFd;       // SCL1: For Object Detection
		MI_S32 m_s32StreamReIdFd;   // SCL2: For ReID

		MI_U16 m_oriHeight;         // SCL0 Height
		MI_U16 m_oriWidth;          // SCL0 Width
		MI_U16 m_SCLHeight;       // SCL1 Height
		MI_U16 m_SCLWidth;        // SCL1 Width
		MI_U16 m_reIdHeight;        // SCL2 Height
		MI_U16 m_reIdWidth;         // SCL2 Width

		MI_VPE_PortMode_t  m_stVpePortMode;
		MI_SYS_ChnPort_t   m_stChnOutputPort[3];            // SCL0: VEC, SCL1: OD, SCL2: ReID
		MI_SCL_OutPortParam_t  m_stSclOutputPortParam[3];   // SCL0: VEC, SCL1: OD, SCL2: ReID

		vector<std::pair<int, int>> m_frameSizeList; // Preserve SCL0, SCL2 frame size
													// for bounding box rescaling
		#endif

		// === Thread Management === //
		bool m_threadTerminated = false;
		bool m_threadStarted = false;
		bool m_isDrawImage = false;
		std::thread m_threadShowLog;
		std::thread m_threadDrawImage;
		std::mutex m_mutex;
		std::condition_variable m_condition;


		// === Input Frame === //

		// === Bounding Box === //

		// Output Bounding Boxes from AI
		std::vector<BoundingBox> m_humanBBoxList;
		std::vector<BoundingBox> m_riderBBoxList;
		std::vector<BoundingBox> m_vehicleBBoxList;
		std::vector<BoundingBox> m_stopSignBBoxList;
		std::vector<BoundingBox> m_roadSignBBoxList;

		// Output Bounding Boxes after filtering by ROI
		std::vector<BoundingBox> m_f_humanBBoxList;
		std::vector<BoundingBox> m_f_riderBBoxList;
		std::vector<BoundingBox> m_f_vehicleBBoxList;
		std::vector<BoundingBox> m_f_stopSignBBoxList;
		std::vector<BoundingBox> m_f_roadSignBBoxList;

		// Output Bounding Boxes for Lane Information
		std::vector<BoundingBox> m_vlaBBoxList;
		std::vector<BoundingBox> m_vpaBBoxList;
		std::vector<BoundingBox> m_dlaBBoxList;
		std::vector<BoundingBox> m_dmaBBoxList;
		std::vector<BoundingBox> m_duaBBoxList;
		std::vector<BoundingBox> m_dcaBBoxList;

		// === Objects === //
		std::vector<Object> m_humanObjList;
		std::vector<Object> m_riderObjList;
		std::vector<Object> m_vehicleObjList;
		std::vector<Object> m_trackedObjList;

		// === Object Tracker === //
		ObjectTracker* m_humanTracker;
		ObjectTracker* m_riderTracker;
		ObjectTracker* m_vehicleTracker;

		// === Optical Flow === //
		cv::Mat m_flow;
		DirectionInfo m_egoDirectionInfo;

		// === Lane Line Detection === //
		vector<Point> m_linePointList;
		LaneLineInfo m_laneLineInfo;
		LaneInfo m_currLaneInfo;
		LaneInfo m_prevLaneInfo;
		LaneInfo m_unscale_currLaneInfo;
		LaneInfo m_unscale_prevLaneInfo;
		bool m_isDetectLine = false;
		int m_yVanish = 0;
		bool m_isLeftLineShift = false;
		bool m_isRightLineShift = false;
		float m_leftLineShiftRatio = 0;
		float m_rightLineShiftRatio = 0;
		LaneLineDetection* m_laneLineDet;

		// === Following Distance === //
		float m_focalRescaleRatio = 0;

		// === LDW === //
		LDW* m_ldw;
		bool m_isLaneDeparture = false;
		LaneDetector m_laneDetector;
		int m_ldwWarnCounter = 0;

		// === FCW === //
		FCW* m_fcw;
		bool m_isForwardCollision = false;
		BoundingBox* m_OD_ROI;
		ROI m_rescaleVehicleZone;
		ROI m_rescaleRiderZone;
		ROI m_rescaleHumanROI;
		ROI m_roi;
		BoundingBox* m_roiBBox;

		// === Result === //
		ADAS_Results m_result;
		std::deque<ADAS_DRAW_RESULTS> m_drawResultBuffer;


		// === Display === //
		cv::Mat m_dsp_img;
		cv::Mat m_dsp_imgResize;
		cv::Mat m_dsp_colorLaneMask;
		cv::Mat m_dsp_colorLineMask;
		cv::Mat m_dsp_calibMask; //TODO:
		cv::Mat m_dsp_laneLineResult;
		cv::Mat m_dsp_mergeLineMask; //TODO:
		
		bool m_dsp_laneLineMask = true;
		bool m_dsp_objectDetection = true;
		bool m_dsp_objectTracking = true;
		bool m_dsp_laneLineDetection = true;
		bool m_dsp_laneLineBoxes = true;
		bool m_dsp_vanishingLine = true;
		bool m_dsp_followingDistance = true;
		bool m_dsp_laneDeparture = true;
		bool m_dsp_forwardCollision = true;
		bool m_dsp_warningZone = true;
		bool m_dsp_information = true;
		int m_dsp_maxFrameIdx;

		// === Debug === //
		bool m_dbg_adas = false;
		bool m_dbg_yoloADAS = false;
		bool m_dbg_opticalFlow = false;
		bool m_dbg_laneLineCalib = false;
		bool m_dbg_laneLineDetection = false;
		bool m_dbg_objectDetection = false;
		bool m_dbg_objectTracking = false;
		bool m_dbg_laneDeparture = false;
		bool m_dbg_followingDistance = false;
		bool m_dbg_forwardCollision = false;
		std::string m_dbg_logsDirPath = "";
		std::string m_dbg_imgsDirPath = "";
		std::string m_dbg_rawImgsDirPath = "";
		std::string m_dbg_dateTime = "";
		LoggerManager m_loggerManager;
};
#endif
