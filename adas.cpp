/*
  (C) 2023-2024 Wistron NeWeb Corporation (WNC) - All Rights Reserved

  This software and its associated documentation are the confidential and
  proprietary information of Wistron NeWeb Corporation (WNC) ("Company") and
  may not be copied, modified, distributed, or otherwise disclosed to third
  parties without the express written consent of the Company.

  Unauthorized reproduction, distribution, or disclosure of this software and
  its associated documentation or the information contained herein is a
  violation of applicable laws and may result in severe legal penalties.
*/
#include "json.hpp"
#include <iostream>
#include <fstream>
#ifdef QCS6490
#include "adas.hpp"
#endif
using json = nlohmann::json;
#ifdef SAV837
#include "wnc_adas.hpp"
#define WNC_DEBUG_INFO 0

// int frameID = 1;
// === SGS Libraries === //
// #include "iout.h"
#include "mi_ipu.h"
#include "mi_vpe.h"
#include "mi_scl.h"

#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
// #include "fusion_ipc.h"
// #include "point.h"

MI_PHY _outputaddr[2] = {0};
int    _u32DestSize[2];
int    _u32DestStride[2];
#endif

using namespace std;

#ifdef QCS6490
ADAS::ADAS(std::string configPath)
{
  auto m_logger = spdlog::stdout_color_mt("ADAS");
  m_logger->set_pattern("[%n] [%^%l%$] %v");

  m_logger->info("=================================================");
  m_logger->info("=                   WNC ADAS                    =");
  m_logger->info("=================================================");
  m_logger->info("Version: v{}", ADAS_VERSION);
  m_logger->info("-------------------------------------------------");

  // === initialize parameters === //
  _init(configPath);

  // Create a file rotating logger with 5 MB size max and 3 rotated files
  auto max_size = 1048576 * 5;
  auto max_files = 3;
  string logName = "log.txt";
  string logPath = m_dbg_logsDirPath + "/" + logName;
  auto m_loggerOutput = spdlog::rotating_logger_mt("ADAS_DEBUG", logPath, max_size, max_files);
  m_loggerOutput->flush_on(spdlog::level::info);
  m_loggerOutput->set_pattern("%v");

  if (m_dbg_adas)
    m_logger->set_level(spdlog::level::debug);
  else
    m_logger->set_level(spdlog::level::info);

	// Enable YOLO-ADAS thread
	m_yoloADAS->runThread();
	m_yoloADAS_PostProc->runThread();
};
#endif

#ifdef SAV837
ADAS::ADAS(IPU_DlaInfo_S& stDlaInfo) : CIpuCommon(stDlaInfo)
{
    auto m_logger = spdlog::stdout_color_mt("ADAS");
    m_logger->set_pattern("[%n] [%^%l%$] %v");

    m_logger->info("=================================================");
    m_logger->info("=                   WNC ADAS                    =");
    m_logger->info("=================================================");
    m_logger->info("Version: v{}", ADAS_VERSION);
    m_logger->info("-------------------------------------------------");

    // SCL0 to RTSP
    m_stChnOutputPort[0].eModId    = E_MI_MODULE_ID_SCL;
    m_stChnOutputPort[0].u32DevId  = 3;
    m_stChnOutputPort[0].u32ChnId  = 2;
    m_stChnOutputPort[0].u32PortId = 0;

    // SCL1 to IPU (WNC-ADAS model)
    m_stChnOutputPort[1].eModId    = E_MI_MODULE_ID_SCL;
    m_stChnOutputPort[1].u32DevId  = 3;
    m_stChnOutputPort[1].u32ChnId  = 2;
    m_stChnOutputPort[1].u32PortId = 0;
    m_s32StreamFd                  = -1;

    m_logger->info("Assigned SCL");

    IpuInit(); // Init SGS IPU // wnc add

    _init("/customer/adas/config/config.txt"); // Configurate parameters, default path for device

    // Create a file rotating logger with 5 MB size max and 3 rotated files
    auto   max_size       = 1048576 * 5;
    auto   max_files      = 3;
    string logName        = "log.txt";
    string logPath        = m_dbg_logsDirPath + "/" + logName;
    auto   m_loggerOutput = spdlog::rotating_logger_mt("ADAS_DEBUG", logPath, max_size, max_files);
    m_loggerOutput->flush_on(spdlog::level::info);
    m_loggerOutput->set_pattern("%v");

    if (m_dbg_adas)
        m_logger->set_level(spdlog::level::debug);
    else
        m_logger->set_level(spdlog::level::info);

	// Enable YOLO-ADAS thread
	m_yoloADAS->runThread();
	m_yoloADAS_PostProc->runThread();
};
#endif

ADAS::~ADAS()
{
	#ifdef SAV837
	IpuDeInit();
	#endif
	stopThread();

	delete m_adasConfigReader;
	delete m_config;
	delete m_yoloADAS;
	delete m_yoloADAS_PostProc;
	delete m_humanTracker;
	delete m_riderTracker;
	delete m_vehicleTracker;
	delete m_laneLineDet;
	delete m_ldw;
	delete m_fcw;
	delete m_OD_ROI;
	delete m_roiBBox;

	m_adasConfigReader = nullptr;
	m_config = nullptr;
	m_yoloADAS = nullptr;
	m_yoloADAS_PostProc = nullptr;
	m_humanTracker = nullptr;
	m_riderTracker = nullptr;
	m_vehicleTracker = nullptr;
	m_laneLineDet = nullptr;
	m_ldw = nullptr;
	m_fcw = nullptr;
	m_OD_ROI = nullptr;
	m_roiBBox = nullptr;
};

void ADAS::stopThread()
{
	cout << "m_yoloADAS_PostProc->stopThread()" << endl;
	m_yoloADAS_PostProc->stopThread();

	cout << "m_yoloADAS->stopThread()" << endl;
	m_yoloADAS->stopThread();
}

#ifdef SAV837
MI_S32 WNC_ADAS::IpuGetSclOutputPortParam(void)
{
    // get preview scaler param
    MI_U32 u32Dev   = 3;
    MI_U32 u32Chn   = 2;
    auto   m_logger = spdlog::get("ADAS");

    m_logger->info("[ADAS] MI_SCL_GetOutputPortParam !!!");

    ExecFunc(MI_SCL_GetOutputPortParam(u32Dev, u32Chn, 0, &m_stSclOutputPortParam[0]), MI_SUCCESS);
    ExecFunc(MI_SCL_GetOutputPortParam(u32Dev, u32Chn, 0, &m_stSclOutputPortParam[1]), MI_SUCCESS);
    // ExecFunc(MI_SCL_GetOutputPortParam(u32Dev, u32Chn, 2, &m_stSclOutputPortParam[2]), MI_SUCCESS);

    m_logger->info("[ADAS] MI_SCL_GetOutputPortParam Done!!!");

    return MI_SUCCESS;
};

MI_S32 WNC_ADAS::IpuInit()
{
    auto m_logger = spdlog::get("ADAS");
    m_logger->info("[ADAS] IPUInit !!!");

    IpuGetSclOutputPortParam(); // Get SCL port 0, 1, 2

    // SCL0: RTSP 960 x 960
    m_oriWidth  = m_stSclOutputPortParam[0].stSCLOutputSize.u16Width;
    m_oriHeight = m_stSclOutputPortParam[0].stSCLOutputSize.u16Height;

    // Save frame size of SCL0
    m_frameSizeList.push_back(std::make_pair(m_oriWidth, m_oriHeight));

    // SCL1: WNC-ADAS 320 x 320 TODO: Change
    m_modelWidth  = m_stSclOutputPortParam[1].stSCLOutputSize.u16Width;
    m_modelHeight = m_stSclOutputPortParam[1].stSCLOutputSize.u16Height;

    ExecFunc(IpuGetStreamFd(&m_stChnOutputPort[1], &m_s32StreamFd), MI_SUCCESS);

    int ori_width  = m_stSclOutputPortParam[1].stSCLOutputSize.u16Width;
    int ori_height = m_stSclOutputPortParam[1].stSCLOutputSize.u16Height;

    _u32DestStride[0] = wnc_ALIGN_UP(ori_width, 16) * 4;
    _u32DestSize[0]   = _u32DestStride[0] * ori_height;
    m_logger->info("[ADAS] SCL1 MMA Alloc Size: {}", _u32DestSize[0]);
    return MI_SUCCESS;
};

MI_S32 WNC_ADAS::IpuDeInit()
{
    // SCL1: WNCADAS
    ExecFunc(IpuPutStreamFd(m_s32StreamFd), MI_SUCCESS);
    ExecFunc(IpuPutStreamFd(m_s32StreamFd), MI_SUCCESS); // [WNC workaround] re deinit to try free
    // ExecFunc(MI_SYS_CloseFd(m_s32StreamFd), MI_SUCCESS); // [WNC workaround] re deinit to try free
    MI_SYS_MMA_Free(0, _outputaddr[0]);
    // SCL2: ReID
    // ExecFunc(IpuPutStreamFd(m_s32StreamReIdFd), MI_SUCCESS); //WNC un-used for LI80
    // MI_SYS_MMA_Free(0,_outputaddr[1]); //WNC un-used for LI80
    return MI_SUCCESS;
};
#endif

bool ADAS::_init(std::string configPath)
{
	// Get Date Time
	utils::getDateTime(m_dbg_dateTime);

	// Read ADAS Configuration
	m_config = new ADAS_Config_S();
	m_adasConfigReader = new ADAS_ConfigReader();
	m_adasConfigReader->read(configPath);
	cout << "[ADAS::_init] << Finished reading configuration file" << endl;

	m_config = m_adasConfigReader->getConfig();
    cout << "[ADAS::_init] << Finished setting configurations" << endl;

	#ifdef SAV837
	// Check IPU firmware path
	if (!utils::checkFileExists(m_config->firmwarePath))
    {
        cerr << "IPU firmware path {" << m_config->firmwarePath <<"} not found" << endl; 
        exit(1);
    }
	#endif

	// Create AI model
	if (!utils::checkFileExists(m_config->modelPath))
	{
		cerr << "Model path {" << m_config->modelPath << "} not found" << endl;
        exit(1);
	}

	m_yoloADAS = new YOLOADAS(m_config);
	cout << "[ADAS::_init] << Initialized AI model" << endl;

	// ROI
	_initROI();

	// IO
	m_videoWidth = m_config->frameWidth;
	m_videoHeight = m_config->frameHeight;
	m_segWidth = m_config->semanticWidth;
	m_maskWidth = m_config->maskWidth;

	// #ifdef QCS6490
	// // Update ROI
	// m_videoROIWidth = newXEnd - newXStart;
	// m_videoROIHeight = newYEnd - newYStart;
	// #endif

	// Model Input Size
	m_modelWidth = m_config->modelWidth;
	m_modelHeight = m_config->modelHeight;

	// Processing
	m_frameStep = m_config->procFrameStep;

	// Distance Estimation
	#ifdef QCS6490
	m_focalRescaleRatio = (float)m_videoHeight / (float)m_modelHeight; 
	// After cropping
	#endif

	#ifdef SAV837
	m_focalRescaleRatio = (float)m_videoHeight / (float)m_modelHeight;
	#endif

	// Vanishing Line
	m_yVanish = m_config->yVanish;

	// Lane Line Detection
	m_laneLineDet = new LaneLineDetection(m_config);
	cout << "[ADAS::_init] << Initialized Lane Line Detection module" << endl;

	// Lane Departure Warning
	m_ldw = new LDW(m_config);
	cout << "[ADAS::_init] << Initialized LDW module" << endl;

	// Forward Collision Detection
	m_fcw = new FCW(m_config);

	#ifdef QCS6490
	m_OD_ROI = new BoundingBox(m_videoWidth * 0.15, m_videoHeight * 0.2, m_videoWidth * 0.85,
							   m_videoHeight * 0.8, -1);
	#endif
	#ifdef SAV837
	m_OD_ROI = new BoundingBox(m_videoWidth * 0.15, m_videoHeight * 0.2, m_videoWidth * 0.85, m_videoHeight * 0.8, -1);
	#endif
	m_fcw->setROI(m_roi);
	cout << "[ADAS::_init] << Initialized FCW module" << endl;

	m_yoloADAS_PostProc = new YOLOADAS_POSTPROC(m_config, m_OD_ROI);
	cout << "[ADAS::_init] << Initialized YOLO-ADAS POST_PROC" << endl;

	// Object Traccker
	m_humanTracker = new ObjectTracker(m_config, "human");
	cout << "[ADAS::_init] << Initialized human tracker module" << endl;

	m_riderTracker = new ObjectTracker(m_config, "rider");
	cout << "[ADAS::_init] << Initialized rider tracker module" << endl;

	m_vehicleTracker = new ObjectTracker(m_config, "vehicle");
	cout << "[ADAS::_init] << Initialized vehicle tracker module" << endl;

	m_vehicleTracker->setROI(m_roi);
	m_riderTracker->setROI(m_roi);
	m_humanTracker->setROI(m_roi);
	cout << "[ADAS::_init] << Set up ROI for Tracker modules" << endl;

	std::vector<DataFrame>* fcwDataBufferPtr = m_fcw->getDataBuffer();
	m_vehicleTracker->setDataBuffer(fcwDataBufferPtr);
	m_riderTracker->setDataBuffer(fcwDataBufferPtr);
	m_humanTracker->setDataBuffer(fcwDataBufferPtr);
	cout << "[ADAS::_init] << Initialized Object Tracker modules" << endl;

	_readDebugConfig();          // Debug Configuration
	_readDisplayConfig();        // Display Configuration
	_readShowProcTimeConfig();   // Show Processing Time Configuration

	return ADAS_SUCCESS;
}

bool ADAS::_readDebugConfig()
{
	auto m_logger = spdlog::get("ADAS");

	if (m_config->stDebugConfig.ADAS)
	{
		m_dbg_adas = true;
	}

	if (m_config->stDebugConfig.yoloADAS)
	{
		m_dbg_yoloADAS = true;

		bool showMask = m_config->stDisplayConfig.laneLineMask;
		// m_yoloADAS->debugON(showMask);
		m_yoloADAS_PostProc->debugON(showMask);
	}

	if (m_config->stDebugConfig.laneLineDetection)
	{
		m_dbg_laneLineDetection = true;
		m_laneLineDet->debugON();
	}

	if (m_config->stDebugConfig.objectDetection)
		m_dbg_objectDetection = true;

	if (m_config->stDebugConfig.objectTracking)
	{
		m_dbg_objectTracking = true;
        if (m_config->stDebugConfig.humanTracker)
            m_humanTracker->debugON();
        if (m_config->stDebugConfig.riderTracker)
            m_riderTracker->debugON();
        if (m_config->stDebugConfig.vehicleTracker)
            m_vehicleTracker->debugON();
	}

	if (m_config->stDebugConfig.laneDeparture)
	{
		m_dbg_laneDeparture = true;
		m_ldw->debugON();
	}

	if (m_config->stDebugConfig.forwardCollision)
		m_dbg_forwardCollision = true;

	if (m_config->stDebugConfig.followingDistance)
		m_dbg_followingDistance = true;

	if (m_config->stDebugConfig.saveLogs)
		m_dbg_saveLogs = true;

	if (m_config->stDebugConfig.saveImages)
		m_dbg_saveImages = true;

	if (m_config->stDebugConfig.saveRawImages)
		m_dbg_saveRawImages = true;

	if (m_config->stDebugConfig.logsDirPath != "")
	{
		m_dbg_logsDirPath = m_config->stDebugConfig.logsDirPath + "/" + m_dbg_dateTime;

		if (m_config->stDebugConfig.saveLogs && utils::createDirectories(m_dbg_logsDirPath))
			m_logger->info("Folders created successfully: {}", m_dbg_logsDirPath);
		else
			m_logger->info("Error creating folders: {}", m_dbg_logsDirPath);
	}

	if (m_config->stDebugConfig.imgsDirPath != "")
	{
		m_dbg_imgsDirPath = m_config->stDebugConfig.imgsDirPath + "/" + m_dbg_dateTime;

		if (m_config->stDebugConfig.saveImages && utils::createDirectories(m_dbg_imgsDirPath))
			m_logger->info("Folders created successfully: {}", m_dbg_imgsDirPath);
		else
			m_logger->info("Error creating folders: {}", m_dbg_imgsDirPath);
	}

	if (m_config->stDebugConfig.rawImgsDirPath != "")
	{
		m_dbg_rawImgsDirPath = m_config->stDebugConfig.rawImgsDirPath + "/" + m_dbg_dateTime;

		if (m_config->stDebugConfig.saveRawImages && utils::createDirectories(m_dbg_rawImgsDirPath))
			m_logger->info("Folders created successfully: {}", m_dbg_rawImgsDirPath);
		else
			m_logger->info("Error creating folders: {}", m_dbg_rawImgsDirPath);
	}

	return ADAS_SUCCESS;
}

bool ADAS::_readDisplayConfig()
{
	if (m_config->stDisplayConfig.results)
		m_dsp_results = true;

	if (m_config->stDisplayConfig.laneLineMask)
		m_dsp_laneLineMask = true;

	if (m_config->stDisplayConfig.objectDetection)
		m_dsp_objectDetection = true;

	if (m_config->stDisplayConfig.objectTracking)
		m_dsp_objectTracking = true;

	if (m_config->stDisplayConfig.laneLineDetection)
		m_dsp_laneLineDetection = true;

	if (m_config->stDisplayConfig.vanishingLine)
		m_dsp_vanishingLine = true;

	if (m_config->stDisplayConfig.followingDistance)
		m_dsp_followingDistance = true;

	if (m_config->stDisplayConfig.laneDeparture)
		m_dsp_laneDeparture = true;

	if (m_config->stDisplayConfig.information)
		m_dsp_information = true;

	if (m_config->stDisplayConfig.forwardCollision)
		m_dsp_forwardCollision = true;

	if (m_config->stDisplayConfig.warningZone)
		m_dsp_warningZone = true;

	if (m_config->stDisplayConfig.maxFrameIndex)
		m_dsp_maxFrameIdx = m_config->stDisplayConfig.maxFrameIndex;

	return ADAS_SUCCESS;
}

bool ADAS::_readShowProcTimeConfig()
{
	if (m_config->stShowProcTimeConfig.ADAS)
		m_estimateTime = true;

	if (m_config->stShowProcTimeConfig.yoloADAS)
	{
		m_yoloADAS->showProcTime();
		m_yoloADAS_PostProc->showProcTime();
	}

	if (m_config->stShowProcTimeConfig.laneFinder)
		m_laneLineDet->showProcTime();

	if (m_config->stShowProcTimeConfig.objectTracking)
	{
		m_humanTracker->showProcTime();
		m_riderTracker->showProcTime();
		m_vehicleTracker->showProcTime();
	}

	if (m_config->stShowProcTimeConfig.laneDeparture)
		m_ldw->showProcTime();

	if (m_config->stShowProcTimeConfig.forwardCollision)
		m_fcw->showProcTime();

	return ADAS_SUCCESS;
}

bool ADAS::_initROI()
{
	#ifdef QCS6490
	// For QCS6490, we need to create a new ROI area
	// Compute new ROI's width and height
	// int newFrameWidth = newXEnd - newXStart;
	// int newFrameHeight = newYEnd - newYStart;
	
	// if (newFrameWidth <= 0 || newFrameHeight <= 0)
	// {
	// 	cerr << "Configuration file: endXRatio must larger than startXRatio and endYRatio must larger than startYRatio"
	// 		 << endl;
	// 	exit(1);
	// }

	// Center of the ROI
	// int xCenter = static_cast<int>(m_frameWidth * 0.5);
	// int yCenter = static_cast<int>(m_frameHeight * 0.5);
	int xCenter = static_cast<int>(m_config->modelWidth * 0.5);
	int yCenter = static_cast<int>(m_config->modelHeight * 0.5);

	// Get new height and width value
	int newWidth = static_cast<int>(m_config->modelWidth * 0.15);
	int newHeight = static_cast<int>(m_config->modelHeight * 0.4);
	// int newWidth = static_cast<int>(m_frameWidth * 0.15);
	// int newHeight = static_cast<int>(m_frameHeight * 0.4);

	// Calculate ROI's x boundaries
	int x1 = xCenter - static_cast<int>(newWidth * 0.6);
	int x2 = xCenter + static_cast<int>(newWidth * 0.6);

	// Calculate ROI's y boundaries
	int y1 = yCenter - static_cast<int>(m_config->modelHeight * 0.2);
	int y2 = yCenter + static_cast<int>(m_config->modelHeight * 0.2);
	// int y1 = yCenter - static_cast<int>(newHeight * 0.2);
	// int y2 = yCenter + static_cast<int>(newHeight * 0.2);

	// Create bounding box ROI area
	m_roi.x1 = x1;
	m_roi.y1 = y1;
	m_roi.x2 = x2;
	m_roi.y2 = y2;

 	m_roiBBox = new BoundingBox(m_roi.x1, m_roi.y1, m_roi.x2, m_roi.y2, -1);
	#endif

	#ifdef SAV837
	// Center of the ROI
	int xCenter = static_cast<int>(m_config->modelWidth * 0.5);
	int yCenter = static_cast<int>(m_config->modelHeight * 0.5); // Assuming we are using hardware solution to crop
	// int yCenter = static_cast<int>(m_config->frameHeight * 0.6); Uncropped by any mean, default by Andy
	
	if (xCenter <= 0 || yCenter <= 0)
	{
		cerr << "Configuration file: frameWidth and frameHeight must larger than 0" << endl;
		exit(1);
	}

	// Get new height and new width value and assume that we are using hardware solution
	// NOTE: For SAV837, it is actually not new height or width, rather this value must be the same in the hardware configuration solution file
	int newWidth = static_cast<int>(m_config->modelWidth * 0.15);
	int newHeight = static_cast<int>(m_config->modelHeight * 0.4);

	// Calculate ROI's x boundaries
	int x1 = xCenter - static_cast<int>(newWidth * 0.6);
	int x2 = xCenter + static_cast<int>(newWidth * 0.6);

	// Calculate ROI's y boundaries
	int y1 = yCenter - static_cast<int>(m_config->modelHeight * 0.2);
	int y2 = yCenter + static_cast<int>(m_config->modelHeight * 0.2);

	// Create bounding box ROI area
	m_roi.x1 = x1;
	m_roi.y1 = y1;
	m_roi.x2 = x2;
	m_roi.y2 = y2;

	m_roiBBox = new BoundingBox(m_roi.x1, m_roi.y1, m_roi.x2, m_roi.y2, -1);
	#endif

	return ADAS_SUCCESS;
}

// ============================================
//                   Main
// ============================================
//#ifdef QCS6490

bool ADAS::run(cv::Mat &imgFrame, cv::Mat &resultMat)
{
 	auto m_logger = spdlog::get("ADAS");
	auto time_0 = std::chrono::high_resolution_clock::now();
	auto time_1 = std::chrono::high_resolution_clock::now();

	bool ret = ADAS_SUCCESS;

	if (imgFrame.empty())
	{
		m_logger->warn("Input image is empty");
		return false;
	}

	int newXStart = static_cast<int>(imgFrame.cols * m_config->startXRatio);
	int newXEnd = static_cast<int>(imgFrame.cols * m_config->endXRatio);
	int newYStart = static_cast<int>(imgFrame.rows * m_config->startYRatio);
	int newYEnd = static_cast<int>(imgFrame.rows * m_config->endYRatio);

	// Get Image Frame
	if (m_dsp_results)
	{
		m_dsp_img = imgFrame.clone();
		m_dsp_img = m_dsp_img(cv::Range(newYStart, newYEnd), cv::Range(newXStart, newXEnd));
		cv::resize(m_dsp_img, m_dsp_img, cv::Size(m_config->frameWidth, m_config->frameHeight), cv::INTER_LINEAR);
	}

	// Entry Point
	if (m_frameIdx % m_frameStep == 0)
	{
		m_logger->info("");
		m_logger->info("========================================");
		m_logger->info("Frame Index: {}", m_frameIdx);
		m_logger->info("========================================");

		// Get Image Frame
		m_img = imgFrame.clone();
		m_img = m_img(cv::Range(newYStart, newYEnd), cv::Range(newXStart, newXEnd));
		cv::resize(m_img, m_img, cv::Size(m_config->modelWidth, m_config->modelHeight), cv::INTER_LINEAR);

		// Update YOLO-ADAS frame buffer
		m_yoloADAS->updateInputFrame(m_img);

		// Get last prediction
		YOLOADAS_Prediction pred;
		int predBufferSize = m_yoloADAS->getLastestPrediction(pred);
		if (predBufferSize > 0)
		{
			// Start doing post processing ...
			m_yoloADAS_PostProc->updatePredictionBuffer(pred);
			m_yoloADAS->removeFirstPrediction();

			m_procResult = {};
			int resultBufferSize = m_yoloADAS_PostProc->getLastestResult(m_procResult);

			if (resultBufferSize == 0)
			{
				return false;
			}

			{
				// Get Detected Lane Line Bounding Boxes
				if (!_laneLineDetection())
				{
					m_logger->warn("Detect lane lines failed ...");
					ret = ADAS_FAILURE;
				}

				// Get Detected Bounding Boxes
				if (!_objectDetection())
				{
					m_logger->warn("Detect objects failed ...");
					ret = ADAS_FAILURE;
				}

				// Object Tracking
				if (!_objectTracking())
				{
					m_logger->warn("Track objects failed ...");
					ret = ADAS_FAILURE;
				}

				// Detect Lane Departure Event
				if (!_laneDepartureDetection())
				{
					m_logger->warn("Detect lane departure event failed ...");
					ret = ADAS_FAILURE;
				}

				// Forward Collision Warning
				if (!_forwardCollisionDetection())
				{
					m_logger->warn("Detect forward collision event failed ...");
					ret = ADAS_FAILURE;
				}
			}

			// End of ADAS Tasks
			m_yoloADAS_PostProc->removeFirstResult();

			// Show Results
			_showDetectionResults();

			// Save Results to Debug Logs
			if (m_dbg_saveLogs)
				_saveDetectionResults();

			if (m_estimateTime)
			{
				time_1 = std::chrono::high_resolution_clock::now();
				m_logger->info("");
				m_logger->info("Processing Time: \t{} ms",
								std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count()
								/ (1000.0 * 1000));
			}
		}
	}

	// Draw and Save Results
	if (m_dsp_results && ret == ADAS_SUCCESS)
		_drawResults();

	if (m_dsp_results && m_dbg_saveImages && ret == ADAS_SUCCESS)
		_saveDrawResults();

	// Save Raw Images
	if (m_dbg_saveRawImages)
		_saveRawImages();

	getResultImage(resultMat); // copy resized_dsp_img to resultMat

	// Update frame index
	_updateFrameIndex();

	// ADAS Log with JSON format
	ADAS_Results adasResult;
	getResults(adasResult);
	//"{"frameId": id, "pLeftFar.x": adasResult.pLeftFar.x, }"
	std::vector<BoundingBox> boundingBoxLists[] =
	{
		m_humanBBoxList,
		m_riderBBoxList,
		m_vehicleBBoxList,
		m_roadSignBBoxList,
		m_stopSignBBoxList
	};
	JSON_LOG json_log;
	std::string json_log_str = json_log.JsonLogString(adasResult, 
													  m_config, 
													  boundingBoxLists, 
													  m_trackedObjList, 
													  m_frameIdx);
	cout<<"==========================================================================="<<endl;
	cout<<json_log_str<<endl;
	cout<<"==========================================================================="<<endl;

	return ret;
}
//#endif

#ifdef SAV837
void WNC_ADAS::IpuRunProcess()
{
    auto m_logger = spdlog::get("ADAS");
    // auto time_0   = std::chrono::high_resolution_clock::now();
    // auto time_1   = std::chrono::high_resolution_clock::now();

    bool   FRAME_SUCCESS = ADAS_SUCCESS;
    MI_S32 s32Ret        = 0;

    fd_set            read_fds;
    struct timeval    tv;
    MI_SYS_BufInfo_t  stBufInfo;
    MI_SYS_BUF_HANDLE stBufHandle;

    FD_ZERO(&read_fds);
    FD_SET(m_s32StreamFd, &read_fds);
    wnc_ADAS_Results adasResult;

    tv.tv_sec  = 0;
    tv.tv_usec = 1000 * 1000; // timeout : 1000ms

    s32Ret = select(m_s32StreamFd + 1, &read_fds, NULL, NULL, &tv);
    if (s32Ret <= 0)
    {
        cerr << "SCL No Select !!!" << endl;
        FRAME_SUCCESS = ADAS_FAILURE;
        exit(1);
    }

    if (FD_ISSET(m_s32StreamFd, &read_fds))
    {
        memset(&stBufInfo, 0x0, sizeof(MI_SYS_BufInfo_t));
        if (MI_SUCCESS == MI_SYS_ChnOutputPortGetBuf(&m_stChnOutputPort[1], &stBufInfo, &stBufHandle))
        {
            // Convert buffered image to cv::Mat
            MI_S32 s32Ret = MI_SUCCESS;

            auto time_0 = std::chrono::high_resolution_clock::now();
            auto time_1 = std::chrono::high_resolution_clock::now();

            // Check pixel format
            if ((stBufInfo.stFrameData.ePixelFormat != E_MI_SYS_PIXEL_FRAME_ABGR8888)
                && (stBufInfo.stFrameData.ePixelFormat != E_MI_SYS_PIXEL_FRAME_ARGB8888)
                && (stBufInfo.stFrameData.ePixelFormat != E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420))
            {
                cerr << "ERROR!!! Pixel format is not valid" << endl;
                FRAME_SUCCESS = ADAS_FAILURE;
                exit(1);
            }

            m_logger->debug("Start converting buffered image to cv::Mat");
            _scaleToModelSize((MI_SYS_BufInfo_t*)&stBufInfo); // Entry Point at the end
            m_logger->debug("Finished converting buffered image to cv::Mat");

            if (m_frameIdx_wnc % m_frameStep_wnc == 0)
            {
                // m_logger->info("################################# Size before processing {}",
                // m_yoloADAS->m_midLinePointLists.size());
                m_logger->debug("");
                m_logger->debug("========================================");
                m_logger->debug("Frame Index: {}", m_frameIdx_wnc);
                m_logger->debug("========================================");


				// Update YOLO-ADAS frame buffer
				m_yoloADAS->updateInputFrame(m_img);

                // AI Inference
                if (m_yoloADAS->isInferenceDone())
                {
					// Get last prediction
					YOLOADAS_Prediction pred;
					m_yoloADAS->getLastPrediction(pred);
					pred.isProcessed = true;

					// Start doing post processing ...
					m_yoloADAS_PostProc->run(pred);
			
					// Lane Line Detection
					if (!_laneLineDetection())
					{
						m_logger->warn("Detect lane lines failed ...");
						FRAME_SUCCESS = ADAS_FAILURE;
					}

					// Get Detected Bounding Boxes
					if (!_objectDetection())
					{
						m_logger->warn("Detect objects failed ...");
						FRAME_SUCCESS = ADAS_FAILURE;
					}

					// Object Tracking
					if (!_objectTracking())
					{
						m_logger->warn("Track objects failed ...");
						FRAME_SUCCESS = ADAS_FAILURE;
					}

					// Detect Lane Departure Event
					if (!_laneDepartureDetection())
					{
						m_logger->warn("Detect lane departure event failed ...");
						FRAME_SUCCESS = ADAS_FAILURE;
					}

					// Forward Collision Warning
					if (!_forwardCollisionDetection())
					{
						m_logger->warn("Detect forward collision event failed ...");
						FRAME_SUCCESS = ADAS_FAILURE;
					}

					// Show Results
					_showDetectionResults();

					// Save results to debug logs
					if (m_dbg_saveLogs)
						_saveDetectionResults();

					if (m_estimateTime)
					{
						time_1 = std::chrono::high_resolution_clock::now();
						m_logger->info("");
						m_logger->info("Processing Time: \t{} ms",
									std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count()
										/ (1000.0 * 1000));
                	}
				}
            }
            else
            {
                usleep(100);
            }

            // Draw and Save Results
            if (m_dsp_results && FRAME_SUCCESS == ADAS_SUCCESS)
                _drawResults();

            if (m_dsp_results && m_dbg_saveImages && FRAME_SUCCESS == ADAS_SUCCESS)
                _saveDrawResults();

            if (m_dbg_saveRawImages)
                _saveRawImages();

            FRAME_SUCCESS = ADAS_SUCCESS;

            getResults(adasResult);
            if (adasResult.eventType == ADAS_EVENT_LDW)
                m_logger->debug("Detect LDW Event!");
            if (adasResult.eventType == ADAS_EVENT_FCW)
                m_logger->debug("Detect FCW Event!");
            if (adasResult.eventType == ADAS_EVENT_LDW_FCW)
            {
                m_logger->debug("Detect LDW Event!");
                m_logger->debug("Detect FCW Event!");
            }

            _updateFrameIndex();
            m_logger->debug("Showing IPU Output");
            IpuShowOutput(adasResult);

            if (MI_SUCCESS != MI_SYS_ChnOutputPortPutBuf(stBufHandle))
            {
                cerr << "[OD_Tracking] MI_SYS_ChnOutputPortPutBuf error" << endl;
            }
        }
        else
        {
            m_logger->warn("Failed to fetch system buffer");
            FRAME_SUCCESS = ADAS_FAILURE;
        }
    }
}
#endif

// ============================================
//              Work Flow Functions
// ============================================
#ifdef SAV837
bool WNC_ADAS::_scaleToModelSize(MI_SYS_BufInfo_t* pstBufInfo)
{
    auto           m_logger       = spdlog::get("ADAS");
    int            PixerBytes     = 1;
    MI_U16         u16Height      = 0;
    MI_U32         u32DestStride  = 0;
    MI_U32         u16ModelWidth  = 0;
    MI_U32         u16ModelHeight = 0;
    unsigned char* pSrcImage      = NULL;
    unsigned char* pDstImage      = NULL;

    // Check buffer type
    if (!pstBufInfo || pstBufInfo->eBufType != E_MI_SYS_BUFDATA_FRAME)
    {
        cerr << "ERROR!!! Buffer data type is invalid!" << endl;
        return E_MI_ERR_FAILED;
    }

    // Check image's channel number
    if (m_yoloADAS->m_processedData.imgResizeC != 3)
    {
        cerr << "ERROR!!! Image channel number is not 3!" << endl;
        return E_MI_ERR_FAILED;
    }

    // Locate Virtual address of the Buffer
    pSrcImage = (unsigned char*)pstBufInfo->stFrameData.pVirAddr[0];

    if (!pSrcImage)
    {
        cerr << "ERROR!!! Failed to obtain source image virtual address inside the buffer!" << endl;
        return E_MI_ERR_FAILED;
    }

    u16ModelHeight = m_yoloADAS->m_processedData.imgResizeH;
    u16ModelWidth  = m_yoloADAS->m_processedData.imgResizeW;

    if ((pstBufInfo->stFrameData.ePixelFormat == E_MI_SYS_PIXEL_FRAME_ABGR8888)
        || (pstBufInfo->stFrameData.ePixelFormat == E_MI_SYS_PIXEL_FRAME_ARGB8888))
    {
        PixerBytes    = 4;
        u32DestStride = wnc_ALIGN_UP(u16ModelWidth, 16) * 4;
        u16Height     = pstBufInfo->stFrameData.u16Height;
    }
    else if (pstBufInfo->stFrameData.ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420)
    {
        u32DestStride = wnc_ALIGN_UP(u16ModelWidth, 16);
        u16Height     = pstBufInfo->stFrameData.u16Height * 3 / 2;
    }
	#if WNC_DEBUG_INFO
    m_logger->debug("Model height and Model width {} x {}", u16ModelHeight, u16ModelWidth);
	#endif
    pDstImage = (unsigned char*)m_yoloADAS->m_inputTensorVector.astArrayTensors[0].ptTensorData[0];
    memcpy(pDstImage, pSrcImage, u16ModelWidth * u16ModelHeight * PixerBytes);

    cv::Mat imgFrame = cv::Mat(u16ModelHeight, u16ModelWidth, CV_8UC4, pDstImage);
    // cv::Mat rgbImage(imgFrame.rows, imgFrame.cols, CV_8UC3);
    // cv::cvtColor(imgFrame, rgbImage, cv::COLOR_BGRA2BGR);

    // MI_SYS_FlushInvCache(m_yoloADAS->m_inputTensorVector.astArrayTensors[0].ptTensorData[0], u16ModelWidth *
    // u16ModelHeight * PixerBytes);

    // cv::Mat img = cv::imread("test.jpg", -1);
    // // int rows = img.rows;
    // // int cols = img.cols;
    // // int cx = (cols / 2) + 1;
    // // int c1 = cx - (rows / 2);
    // // int c2 = cx + (rows / 2);
    // // cv::Mat croppedImage = img(cv::Range(0, rows), cv::Range(c1, c2));
    // cv::Size inputSize = cv::Size(u16ModelWidth, u16ModelHeight);
    // cv::Mat ResizedImg;
    // cv::Mat Sample;
    // cv::resize(img, ResizedImg, inputSize);
    // // if (ResizedImg.empty())
    // m_logger->info("REACHED");
    // cv::cvtColor(ResizedImg, Sample, cv::COLOR_BGR2BGRA);
    // // cv::namedWindow("Car",cv::WINDOW_AUTOSIZE);
    // cv::imwrite("test_output.jpg", Sample);
    // m_logger->info("PASSED");
    // // // cv::waitKey(3000);
    // // // cv::destroyWindow("Car");

    // memcpy(pDstImage, Sample.data, u16ModelWidth * u16ModelHeight * 4);

    if (m_dsp_results)
        // TODO: cv::cvtColor(imgFrame, m_dsp_img, cv::COLOR_RGBA2RGB);
        m_dsp_img = imgFrame.clone();

    // Entry Point
    m_img = imgFrame.clone();

    memcpy(pDstImage, imgFrame.data, u16ModelWidth * u16ModelHeight * 4);
    // cv::imwrite("input.jpg", m_img); // wnc modify to check input image

    // m_logger->info("FINISHED");
    return MI_SUCCESS;
}
#endif


bool ADAS::_modelInfernece()
{
	auto m_logger = spdlog::get("ADAS");

	if (!m_yoloADAS->run(m_img))
		return ADAS_FAILURE;

	return ADAS_SUCCESS;
}

//TODO: @QuangDao, we need to implement a new LDW
bool ADAS::_laneDepartureDetection()
{
	auto m_logger = spdlog::get("ADAS");

	if (m_isDetectLine)
	{
		// m_isLeftLineShift = m_yoloADAS->m_laneLineCalib->detectLeftLineShift(m_leftLineShiftRatio);
		// m_isRightLineShift = m_yoloADAS->m_laneLineCalib->detectRightLineShift(m_rightLineShiftRatio);

		// LineInfo leftLine = m_yoloADAS->m_laneLineCalib->m_leftLineInfo;
		// LineInfo rightLine = m_yoloADAS->m_laneLineCalib->m_rightLineInfo;

		// m_isLeftLineShift = m_yoloADAS_PostProc->m_laneLineCalib->detectLeftLineShift(m_leftLineShiftRatio);
		// m_isRightLineShift = m_yoloADAS_PostProc->m_laneLineCalib->detectRightLineShift(m_rightLineShiftRatio);

		// LineInfo leftLine = m_yoloADAS_PostProc->m_laneLineCalib->m_leftLineInfo;
		// LineInfo rightLine = m_yoloADAS_PostProc->m_laneLineCalib->m_rightLineInfo;

		// m_isLeftLineShift = m_procResult.isLeftLineShift;
		// m_isRightLineShift = m_procResult.isRightLineShift;
		// m_leftLineShiftRatio = m_procResult.leftLineShiftRatio;
		// m_rightLineShiftRatio = m_procResult.rightLineShiftRatio;
		// LineInfo leftLine = m_procResult.leftLine;
		// LineInfo rightLine = m_procResult.rightLine;

		m_ldw->enable();
		// m_isLaneDeparture = m_ldw->run(m_currLaneInfo, m_laneLineInfo, m_roadSignBBoxList, m_linePointList,
		// 													m_egoDirectionInfo.directionStr);
	}
	else
	{
		m_ldw->disable();
		m_ldwWarnCounter = m_ldw->getWarnCounter();

		if (((float)m_laneLineInfo.maxLaneWidth / (float)m_modelWidth) > 0.55)
		{
			m_isLaneDeparture = false;
		}
		else if (m_ldwWarnCounter == 0)
		{
			m_isLaneDeparture = false;
		}
	}

	// Update Lane Information
	m_unscale_prevLaneInfo = m_unscale_currLaneInfo;

	// Debug Lane Departure Detector
	if (m_dbg_laneDeparture)
	{
		m_ldw->getLaneDetector(m_laneDetector, m_img.cols, m_img.rows);
	}

	return ADAS_SUCCESS;
}


bool ADAS::_laneLineDetection()
{
	auto m_logger = spdlog::get("ADAS");
	m_vlaBBoxList = m_procResult.vlaBBoxList;
	m_vpaBBoxList = m_procResult.vpaBBoxList;
	m_dlaBBoxList = m_procResult.dlaBBoxList;
	m_dmaBBoxList = m_procResult.dmaBBoxList;
	m_duaBBoxList = m_procResult.duaBBoxList;
	m_dcaBBoxList = m_procResult.dcaBBoxList;

	// Debug Logs
	m_logger->debug("Num of VLA Bounding Box: {} / {}",
	(int)m_humanBBoxList.size(), (int)m_f_humanBBoxList.size());

	m_logger->debug("Num of VPA Bounding Box: {} / {}",
	(int)m_riderBBoxList.size(), (int)m_f_riderBBoxList.size());

	m_logger->debug("Num of DLA Bounding Box: {} / {}",
	(int)m_vehicleBBoxList.size(), (int)m_f_vehicleBBoxList.size());

	m_logger->debug("Num of DMA Bounding Box: {} / {}",
	(int)m_roadSignBBoxList.size(), (int)m_roadSignBBoxList.size());

	m_logger->debug("Num of DUA Bounding Box: {} / {}",
	(int)m_stopSignBBoxList.size(), (int)m_stopSignBBoxList.size());

	m_logger->debug("Num of DCA Bounding Box: {} / {}",
	(int)m_stopSignBBoxList.size(), (int)m_stopSignBBoxList.size());

	//
	LaneLineBoxes laneLineBox = {};
	laneLineBox.vlaBBoxList.insert(laneLineBox.vlaBBoxList.end(), m_vlaBBoxList.begin(), m_vlaBBoxList.end());
	laneLineBox.vpaBBoxList.insert(laneLineBox.vpaBBoxList.end(), m_vpaBBoxList.begin(), m_vpaBBoxList.end());
	laneLineBox.duaBBoxList.insert(laneLineBox.duaBBoxList.end(), m_duaBBoxList.begin(), m_duaBBoxList.end());
	laneLineBox.dmaBBoxList.insert(laneLineBox.dmaBBoxList.end(), m_dmaBBoxList.begin(), m_dmaBBoxList.end());
	laneLineBox.dlaBBoxList.insert(laneLineBox.dlaBBoxList.end(), m_dlaBBoxList.begin(), m_dlaBBoxList.end());
	laneLineBox.dcaBBoxList.insert(laneLineBox.dcaBBoxList.end(), m_dcaBBoxList.begin(), m_dcaBBoxList.end());

	//
	m_currLaneInfo = m_laneLineDet->find(laneLineBox, m_laneLineInfo);
	m_isDetectLine = m_laneLineDet->isDetectLine();
	m_logger->debug("pLeftCarhood = ({}, {})", m_currLaneInfo.pLeftCarhood.x, m_currLaneInfo.pLeftCarhood.y);
	m_logger->debug("pLeftFar = ({}, {})", m_currLaneInfo.pLeftFar.x, m_currLaneInfo.pLeftFar.y);
	m_logger->debug("pRightCarhood = ({}, {})", m_currLaneInfo.pRightCarhood.x, m_currLaneInfo.pRightCarhood.y);
	m_logger->debug("pRightFar = ({}, {})", m_currLaneInfo.pRightFar.x, m_currLaneInfo.pRightFar.x);
	m_logger->debug("pLeftDegree = ({})", m_currLaneInfo.leftDegree);
	m_logger->debug("pRightDegree = ({})", m_currLaneInfo.rightDegree);

	// Update Vanish Line Y when detect lane lines
	int tmpVanishY = m_laneLineDet->getVanishY();
	if (tmpVanishY != 0)
	{
		m_yVanish = tmpVanishY;
		m_yVanish = std::round(m_yVanish * m_focalRescaleRatio);
	}

	m_fcw->setDetectLines(m_isDetectLine);

	if (m_isDetectLine)
	{
		// Update Detection Zone
		m_fcw->setZone(m_currLaneInfo);
		m_fcw->updateDefaultZone(m_currLaneInfo);

		// Direction Vector
		vector<cv::Point> dirVec(2);
		utils::rescalePoint(
				m_laneLineInfo.drivingDirectionVector[0], dirVec[0],
				m_config->modelWidth, m_config->modelHeight,
				m_videoWidth, m_videoHeight);

		utils::rescalePoint(
				m_laneLineInfo.drivingDirectionVector[1], dirVec[1],
				m_config->modelWidth, m_config->modelHeight,
				m_videoWidth, m_videoHeight);

		m_fcw->setLaneDirectionVector(dirVec[1], dirVec[0]);

		// Vanish line
		m_fcw->setVanishLine(m_yVanish);

		// Lane Head Boundary
		m_fcw->setLaneHeadBoundary(m_currLaneInfo.pLeftFar.x, m_currLaneInfo.pRightFar.x);
	}
	else
	{
		//
		LaneInfo pseudoLaneInfo = {};
		m_laneLineDet->getPseudoLaneInfo(pseudoLaneInfo);
		m_fcw->setZone(pseudoLaneInfo);
	}

	// Debug Logs
	m_logger->debug("Detect lane lines = {}", m_isDetectLine);
	m_logger->debug("Y of vanishing line = {}", m_yVanish);

	return ADAS_SUCCESS;
}


bool ADAS::_objectDetection()
{
	auto m_logger = spdlog::get("ADAS");
	m_humanBBoxList = m_procResult.humanBBoxList;
	m_riderBBoxList = m_procResult.riderBBoxList;
	m_vehicleBBoxList = m_procResult.vehicleBBoxList;
	m_roadSignBBoxList = m_procResult.roadSignBBoxList;
	m_stopSignBBoxList = m_procResult.stopSignBBoxList;

	//
	if (!m_isDetectLine && m_fcw->isCarRightAheadCloserThanMeters(m_vehicleBBoxList, 50))
	{
		cout << "isCarRightAheadCloserThanMeters" << endl;
	}

	// if too close to front car, camera cannot see lane line
	// so we use pseudo lane mask
	if (m_fcw->isCarAheadCloserThanMeters(m_vehicleBBoxList, 10))
	{
		//
		LaneInfo pseudoLaneInfo = {};
		m_laneLineDet->getPseudoLaneInfo(pseudoLaneInfo);
		m_fcw->setZone(pseudoLaneInfo);
		m_isDetectLine = false;
	}

	//
	m_fcw->humanBoxFilter(m_humanBBoxList, m_f_humanBBoxList);
	m_fcw->riderBoxFilter(m_riderBBoxList, m_f_riderBBoxList);
	m_fcw->vehicleBoxFilter(m_vehicleBBoxList, m_f_vehicleBBoxList);

	// Debug Logs
	m_logger->debug("Num of Human Bounding Box: {} / {}",
	(int)m_humanBBoxList.size(), (int)m_f_humanBBoxList.size());

	m_logger->debug("Num of Rider Bounding Box: {} / {}",
	(int)m_riderBBoxList.size(), (int)m_f_riderBBoxList.size());

	m_logger->debug("Num of Vehicle Bounding Box: {} / {}",
	(int)m_vehicleBBoxList.size(), (int)m_f_vehicleBBoxList.size());

	m_logger->debug("Num of Road Sign Bounding Box: {} / {}",
	(int)m_roadSignBBoxList.size(), (int)m_roadSignBBoxList.size());

	m_logger->debug("Num of Stop Sign Bounding Box: {} / {}",
	(int)m_stopSignBBoxList.size(), (int)m_stopSignBBoxList.size());


	return ADAS_SUCCESS;
}


bool ADAS::_objectTracking()
{
	auto m_logger = spdlog::get("ADAS");
	// Update FCW Data Buffer for Calculating TTC Before Tracking Objects
	m_fcw->updateDataBuffer(m_img, *m_roiBBox);

	// Run Object Tracking
	m_humanTracker->run(m_img, m_f_humanBBoxList, m_yVanish, m_roi);
	m_riderTracker->run(m_img, m_f_riderBBoxList, m_yVanish, m_roi);
	m_vehicleTracker->run(m_img, m_f_vehicleBBoxList, m_yVanish, m_roi);

	// Get Tracked Objects
	m_humanTracker->getObjectList(m_humanObjList);
	m_riderTracker->getObjectList(m_riderObjList);
	m_vehicleTracker->getObjectList(m_vehicleObjList);

	// Merge Tracked Objects
	m_trackedObjList.clear();
	m_trackedObjList.insert(m_trackedObjList.end(), m_humanObjList.begin(), m_humanObjList.end());
	m_trackedObjList.insert(m_trackedObjList.end(), m_riderObjList.begin(), m_riderObjList.end());
	m_trackedObjList.insert(m_trackedObjList.end(), m_vehicleObjList.begin(), m_vehicleObjList.end());

	// Debug Logs
	m_logger->debug("Track: {} Human, {} Rider, {} Vehicle",
	(int)m_humanObjList.size(), (int)m_riderObjList.size(), (int)m_vehicleObjList.size());

	return ADAS_SUCCESS;
}


bool ADAS::_forwardCollisionDetection()
{
	auto m_logger = spdlog::get("ADAS");

	m_isForwardCollision = m_fcw->run(m_img, m_trackedObjList, m_yVanish);

	return ADAS_SUCCESS;
}

// ============================================
//                  Outputs
// ============================================
void ADAS::getResults(ADAS_Results &res)
{
	utils::rescaleLine(
	m_fcw->m_vehicleZone, m_rescaleVehicleZone,
	m_config->modelWidth, m_config->modelHeight,
	m_config->frameWidth, m_config->frameHeight);

	m_result.isDetectLine = m_isDetectLine;

	if (m_rescaleVehicleZone.pLeftFar.y != 0 && m_rescaleVehicleZone.pRightFar.y != 0)
	{
		m_result.pLeftFar = m_rescaleVehicleZone.pLeftFar;
		m_result.pLeftCarhood = m_rescaleVehicleZone.pLeftCarhood;
		m_result.pRightFar = m_rescaleVehicleZone.pRightFar;
		m_result.pRightCarhood = m_rescaleVehicleZone.pRightCarhood;
	}

	// Save Vanishing Line's Y
	m_result.yVanish = m_yVanish;

	// Save Event Results
	m_result.eventType = getDetectEvents();

	// Save Tracked Objects
	m_result.objList = m_trackedObjList;

	// Pass reference to res
	res = m_result;
}

int ADAS::getDetectEvents()
{
	if (m_isLaneDeparture & m_isForwardCollision)
	{
		return ADAS_EVENT_LDW_FCW;
	}
	else if (m_isLaneDeparture)
	{
		return ADAS_EVENT_LDW;
	}
	else if (m_isForwardCollision)
	{
		return ADAS_EVENT_FCW;
	}
	else
	{
		return ADAS_EVENT_NORMAL;
	}
}

float ADAS::getFollowingDistance()
{
	// TODO: need to improve
	int maxArea = 0;
	float followDistance = -1;

	for (int i = 0; i < m_vehicleObjList.size(); i++)
	{
		Object& obj = m_vehicleObjList[i];
		if (obj.bbox.getArea() > maxArea)
		{
			maxArea = obj.bbox.getArea();
			followDistance = obj.distanceToCamera;
		}
	}

	return followDistance;
}

void ADAS::getResultImage(cv::Mat &imgResult)
{
	if (m_dsp_results)
		imgResult = m_dsp_imgResize;
}

// ============================================
//                  Results
// ============================================

void ADAS::_showDetectionResults()
{
	auto m_logger = spdlog::get("ADAS");

	// Ego Direction
	m_logger->debug("Ego Direction = {}", m_egoDirectionInfo.directionStr);

	// Lane Lines Information
	m_logger->debug("Detect Lane Lines = {:}", m_currLaneInfo.numCarhood);
	m_logger->debug("Left Line Degree = {:.2f}", m_currLaneInfo.leftDegree);
	m_logger->debug("Right Line Degree = {:.2f}", m_currLaneInfo.rightDegree);
	m_logger->debug("Vanishing Line y = {:}", m_yVanish);

	// Lane Departure Warning
	m_logger->debug("Detect LDW Event = {}", m_isLaneDeparture);

	// Forward Collision Warning
	m_logger->debug("Detect FCW Event = {}", m_isForwardCollision);

	// Show Tracked Object Information
	for (int i = 0; i < m_trackedObjList.size(); i++)
	{
		const Object& obj = m_trackedObjList[i];

		if (obj.status == 1)
		{
			string classType = "";
			if (obj.bbox.label == HUMAN)
				classType = "Pedestrian";
			else if (obj.bbox.label == SMALL_VEHICLE)
				classType = "Rider";
			else if (obj.bbox.label == BIG_VEHICLE)
				classType = "Vehicle";

			float ttc = obj.currTTC;

			if (!obj.needWarn)
				ttc = NAN;

			m_logger->debug("Tracking Obj[{}]: Cls = {}, Conf = {:.2f}, Dist = {:.2f}, TTC = {:.2f}, needWarn = {}",
			obj.id, classType, obj.bbox.confidence, obj.distanceToCamera, ttc, obj.needWarn);
		}
	}
}

void ADAS::_saveDetectionResult(std::vector<std::string>& logs)
{
	auto m_logger = spdlog::get("ADAS_DEBUG");

	for (int i = 0; i < logs.size(); i++)
	{
		m_logger->info(logs[i]);
	}
}

void ADAS::_saveDetectionResults()
{
	auto m_logger = spdlog::get("ADAS_DEBUG");

	m_logger->info("");
	m_logger->info("=================================");
	m_logger->info("Frame Index:{}", m_frameIdx);
	m_logger->info("=================================");

	// --- Lane and Line Mask ---  //
	m_loggerManager.m_laneMaskLogger->logResult(m_laneLineInfo.laneMaskInfo);
	m_loggerManager.m_lineMaskLogger->logResult(m_laneLineInfo.lineMaskInfo);

	// --- Ego Direction --- //
	m_loggerManager.m_egoDirectionLogger->logDirectionInfo(m_egoDirectionInfo);

	m_logger->info("");
	m_logger->info("Ego Direction");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_egoDirectionLogger->m_logs);

	// --- Lane Line Detection --- //
	m_loggerManager.m_lineDetetionLogger->logReulst(m_currLaneInfo);
	m_logger->info("");
	m_logger->info("Line Detection");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_lineDetetionLogger->m_logs);

	// --- Object Detection --- //
	std::vector<BoundingBox> bboxList;

	bboxList.insert(bboxList.end(), m_humanBBoxList.begin(), m_humanBBoxList.end());
	bboxList.insert(bboxList.end(), m_riderBBoxList.begin(), m_riderBBoxList.end());
	bboxList.insert(bboxList.end(), m_vehicleBBoxList.begin(), m_vehicleBBoxList.end());
	bboxList.insert(bboxList.end(), m_roadSignBBoxList.begin(), m_roadSignBBoxList.end());
	bboxList.insert(bboxList.end(), m_stopSignBBoxList.begin(), m_stopSignBBoxList.end());

	m_loggerManager.m_objectDetectionLogger->logObjects(bboxList);
	m_logger->info("");
	m_logger->info("Object Detection");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_objectDetectionLogger->m_logs);

	// --- Object Tracking --- //
	m_loggerManager.m_objectTrackingLogger->logObjects(m_trackedObjList);

	m_logger->info("");
	m_logger->info("Object Tracking");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_objectTrackingLogger->m_logs);

	// --- Lane Departure --- //
	LaneDepartureInfo ldwInfo;
	m_ldw->getLDWInfo(ldwInfo);
	m_loggerManager.m_laneDepartureWarningLogger->logResult(ldwInfo);

	m_logger->info("");
	m_logger->info("Lane Departure Warning");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_laneDepartureWarningLogger->m_logs);

	// --- Forward Collision --- //
	ForwardCollisionInfo fcwInfo;
	m_fcw->getFCWInfo(fcwInfo);
	m_loggerManager.m_forwardCollisionWarningLogger->logResult(fcwInfo);

	m_logger->info("");
	m_logger->info("Forward Collision Warning");
	m_logger->info("---------------------------------");
	_saveDetectionResult(m_loggerManager.m_forwardCollisionWarningLogger->m_logs);
}

void ADAS::_saveDrawResults()
{
	auto m_logger = spdlog::get("ADAS");

	string imgName = "frame_" + std::to_string(m_frameIdx) + ".jpg";
	string imgPath = m_dbg_imgsDirPath + "/" + imgName;

	cv::imwrite(imgPath, m_dsp_imgResize);
	m_logger->debug("Save img to {}", imgPath);
}

void ADAS::_saveRawImages()
{
	auto m_logger = spdlog::get("ADAS");

	string imgName = "frame_" + std::to_string(m_frameIdx) + ".jpg";
	string imgPath = m_dbg_rawImgsDirPath + "/" + imgName;

	cv::imwrite(imgPath, m_img);
	m_logger->debug("Save raw img to {}", imgPath);
}

// ============================================
//               Draw Results
// ============================================

void ADAS::_drawLDWROI()
{
	float scale_ratio = m_dsp_img.cols / m_maskWidth;
	scale_ratio = m_dsp_img.cols / m_segWidth;
	int middle_x = (int)(m_laneLineInfo.laneMaskInfo.xCenterAdjust * scale_ratio);
	cv::line(m_dsp_img, cv::Point(middle_x, 0), cv::Point(middle_x, m_dsp_img.rows - 1), cv::Scalar(255, 255, 255), 2);
}


void ADAS::_drawLaneLineBoundingBoxes()
{
	std::vector<BoundingBox> boundingBoxLists[] =
	{
		// m_vlaBBoxList,
		// m_vpaBBoxList,
		m_dlaBBoxList,
		m_dmaBBoxList,
		m_duaBBoxList,
		m_dcaBBoxList
	};

	cv::Scalar colors[] =
	{
		cv::Scalar(204, 0, 102),   // Purple for VLA
		cv::Scalar(0, 0, 204),     // Red for PCA
		cv::Scalar(204, 0, 0),     // DLA
		cv::Scalar(255, 128, 0),   // DMA
		cv::Scalar(255, 255, 0),   // DUA
		cv::Scalar(128, 255, 0)    // DCA
	};

	for (int j = 0; j < sizeof(boundingBoxLists) / sizeof(boundingBoxLists[0]); j++)
	{
		std::vector<BoundingBox>& boundingBoxList = boundingBoxLists[j];
		cv::Scalar color = colors[j];

		for (int i = 0; i < boundingBoxList.size(); i++)
		{
			BoundingBox lastBox = boundingBoxList[i];
			BoundingBox rescaleBox(-1, -1, -1, -1, -1);

			utils::rescaleBBox(
			lastBox, rescaleBox,
			m_config->modelWidth, m_config->modelHeight,
			m_config->frameWidth, m_config->frameHeight);

			imgUtil::roundedRectangle(
				m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
				cv::Point(rescaleBox.x2, rescaleBox.y2),
				color, 2, 0, 10, false);
		}
	}
}

void ADAS::_drawBoundingBoxes()
{
	std::vector<BoundingBox> boundingBoxLists[] =
	{
		m_humanBBoxList,
		m_riderBBoxList,
		m_vehicleBBoxList,
		m_roadSignBBoxList,
		m_stopSignBBoxList};

	cv::Scalar colors[] =
	{
		cv::Scalar(0, 255, 0),     // Green for vehicles
		cv::Scalar(255, 0, 255),   // Magenta for riders
		cv::Scalar(0, 128, 255),   // Orange for humans
		cv::Scalar(255, 255, 0),   // Yellow for road signs
		cv::Scalar(196, 62, 255)   // Purple for stop signs
	};

	for (int j = 0; j < sizeof(boundingBoxLists) / sizeof(boundingBoxLists[0]); j++)
	{
		std::vector<BoundingBox>& boundingBoxList = boundingBoxLists[j];
		cv::Scalar color = colors[j];

		for (int i = 0; i < boundingBoxList.size(); i++)
		{
			BoundingBox lastBox = boundingBoxList[i];
			BoundingBox rescaleBox(-1, -1, -1, -1, -1);

			utils::rescaleBBox(
			lastBox, rescaleBox,
			m_config->modelWidth, m_config->modelHeight,
			m_config->frameWidth, m_config->frameHeight);

			imgUtil::roundedRectangle(
				m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
				cv::Point(rescaleBox.x2, rescaleBox.y2),
				color, 2, 0, 10, false);
		}
	}
}

void ADAS::_drawTrackedObjects()
{
	// Drawing zones if dsp_warningZone is true
	if (m_dsp_warningZone)
	{
		ROI fcw_vehicle_roi;
		ROI fcw_rider_roi;
		ROI fcw_human_roi;

		utils::rescaleROI(
		m_fcw->m_vehicleROI, fcw_vehicle_roi,
		m_config->modelWidth, m_config->modelHeight,
		m_videoWidth, m_videoHeight);

		imgUtil::roundedRectangle(
			m_dsp_img,
			cv::Point(fcw_vehicle_roi.x1, fcw_vehicle_roi.y1),
			cv::Point(fcw_vehicle_roi.x2, fcw_vehicle_roi.y2),
			cv::Scalar(255, 255, 255),
			2, 0, 10, false);

		utils::rescaleROI(
		m_fcw->m_riderROI, fcw_rider_roi,
		m_config->modelWidth, m_config->modelHeight,
		m_videoWidth, m_videoHeight);

		imgUtil::roundedRectangle(
			m_dsp_img,
			cv::Point(fcw_rider_roi.x1, fcw_rider_roi.y1),
			cv::Point(fcw_rider_roi.x2, fcw_rider_roi.y2),
			cv::Scalar(0, 128, 255),
			2, 0, 10, false);

		utils::rescaleROI(
		m_fcw->m_humanROI, fcw_human_roi,
		m_config->modelWidth, m_config->modelHeight,
		m_videoWidth, m_videoHeight);

		imgUtil::roundedRectangle(
			m_dsp_img,
			cv::Point(fcw_human_roi.x1, fcw_human_roi.y1),
			cv::Point(fcw_human_roi.x2, fcw_human_roi.y2),
			cv::Scalar(0, 0, 255),
			2, 0, 10, false);
	}

	for (int i = 0; i < m_trackedObjList.size(); i++)
	{
		const Object& trackedObj = m_trackedObjList[i];

		// Skip objects with specific conditions
		if (trackedObj.status == 0 || trackedObj.disappearCounter > 5 || trackedObj.bboxList.empty())
			continue;
		// if (trackedObj.bboxList.empty())
		// 	continue;

		BoundingBox lastBox = trackedObj.bboxList.back();
		BoundingBox rescaleBox(-1, -1, -1, -1, -1);

		// Rescale BBoxes
		#ifdef SAV837
		utils::rescaleBBox(
		lastBox, rescaleBox,
		m_config->modelWidth, m_config->modelHeight,
		m_config->frameWidth, m_config->frameHeight);
		#endif

		#ifdef QCS6490
		utils::rescaleBBox(
		lastBox, rescaleBox,
		m_config->modelWidth, m_config->modelHeight,
		m_config->frameWidth, m_config->frameHeight);
		#endif

		if (trackedObj.aliveCounter < 3)
		{
			imgUtil::efficientRectangle(
			m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
			cv::Point(rescaleBox.x2, rescaleBox.y2),
			cv::Scalar(0, 250, 0), 2, 0, 10, false);
		}
		else
		{
			cv::Scalar color;
			if (lastBox.label == 0) // Human
				color = cv::Scalar(255, 51, 153);  // Purple 
			else if (lastBox.label == 1) // Rider
				color = cv::Scalar(255, 51, 255);  // Pink
			else if (lastBox.label == 2) // Vehicle
				color = cv::Scalar(255, 153, 153); // Purple Blue

			imgUtil::efficientRectangle(
			m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
			cv::Point(rescaleBox.x2, rescaleBox.y2),
			color, 2, 0, 10, false);
		}

		// Handle Distance Display
		if (m_dsp_followingDistance)
		{
			float distance = trackedObj.distanceToCamera;
			if (distance >= 0 && !std::isnan(distance)) // Show distance if it is positive 
			{
				// Text box
				cv::rectangle(m_dsp_img, cv::Point(rescaleBox.x1 + 10, rescaleBox.y1 - 20),
							  cv::Point(rescaleBox.x1 + 40, rescaleBox.y1 - 3), (0, 0, 0), -1
							  /*fill*/);

				cv::putText(m_dsp_img, std::to_string(trackedObj.id) + ":" + std::to_string((int)distance) + "m",
							cv::Point(int(rescaleBox.x1) + 10, int(rescaleBox.y1) - 10), cv::FONT_HERSHEY_DUPLEX,
							0.3, cv::Scalar(255, 255, 255), 1, 5, 0);
			}
		}

		if (m_dsp_forwardCollision)
		{
			float ttc = trackedObj.currTTC;

			// Handle TTC display
			std::string ttcString = utils::to_string_with_precision((float)ttc, 1);
			float ttcTH = m_config->stFcwConfig.ttc;

			if (trackedObj.ttcCounter > 0 && ttc < ttcTH)
			{
				// Draw bounding box in red
				imgUtil::efficientRectangle(
				m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
				cv::Point(rescaleBox.x2, rescaleBox.y2),
				cv::Scalar(0, 0, 255), 2, 0, 10, false);

				// Draw TTC information
				cv::rectangle(
				m_dsp_img,
				cv::Point(rescaleBox.x1 + 10, rescaleBox.y2 + 3),
				cv::Point(rescaleBox.x1 + 88, rescaleBox.y2 + 35),
				(0, 0, 0),
				-1/*fill*/);

				cv::putText(m_dsp_img, ttcString,
				cv::Point(int(rescaleBox.x1) + 20, int(rescaleBox.y2) + 28),
				cv::FONT_HERSHEY_DUPLEX, 0.6,
				cv::Scalar(255, 255, 255), 1, 5, 0);
			}
		}
	}
}

#ifdef SAV837
//TODO: need to optimized
//[WNC fixed m_dsp_calibMask format to CV_8UC4 from CV_8UC3]
cv::Mat convertRGBToARGB(const cv::Mat& rgbImage)
{
    int height = rgbImage.rows;
    int width  = rgbImage.cols;

    cv::Mat argbImage = cv::Mat::zeros(height, width, CV_8UC4);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            cv::Vec3b  rgbPixel  = rgbImage.at<cv::Vec3b>(y, x);
            cv::Vec4b& argbPixel = argbImage.at<cv::Vec4b>(y, x);

            argbPixel[0] = rgbPixel[2]; // R channel
            argbPixel[1] = rgbPixel[1]; // G channel
            argbPixel[2] = rgbPixel[0]; // B channel
            argbPixel[3] = 255;         // Alpha channel
        }
    }

    return argbImage;
}
#endif


void ADAS::_drawLaneLines()
{
	#ifdef SAV837
	utils::rescaleLine(
		m_fcw->m_vehicleZone, m_rescaleVehicleZone,
		m_config->modelWidth, m_config->modelHeight,
		m_config->frameWidth, m_config->frameHeight);
	#endif

	#ifdef QCS6490
	utils::rescaleLine(
		m_fcw->m_vehicleZone, m_rescaleVehicleZone,
		m_config->modelWidth, m_config->modelHeight,
		m_config->frameWidth, m_config->frameHeight);
	#endif

	m_dsp_laneLineResult = cv::Mat::zeros(m_dsp_img.rows, m_dsp_img.cols, CV_8UC3); // QD: Change to m_dsp_img for avoiding manual change

	// cout << "m_videoROIWidth = " << m_videoROIWidth << endl;
	// cout << "m_videoROIHeight = " << m_videoROIHeight << endl;
	// cout << "m_rescaleVehicleZone.pLeftFar.x = " << m_rescaleVehicleZone.pLeftFar.x << endl;
	// cout << "m_rescaleVehicleZone.pLeftFar.y = " << m_rescaleVehicleZone.pLeftFar.y << endl;
	// cout << "m_rescaleVehicleZone.pLeftCarhood.x = " << m_rescaleVehicleZone.pLeftCarhood.x << endl;
	// cout << "m_rescaleVehicleZone.pLeftCarhood.y = " << m_rescaleVehicleZone.pLeftCarhood.y << endl;
	// cout << "m_rescaleVehicleZone.pRightFar.x = " << m_rescaleVehicleZone.pRightFar.x << endl;
	// cout << "m_rescaleVehicleZone.pRightFar.y = " << m_rescaleVehicleZone.pRightFar.y << endl;
	// cout << "m_rescaleVehicleZone.pRightCarhood.x = " << m_rescaleVehicleZone.pRightCarhood.x << endl;
	// cout << "m_rescaleVehicleZone.pRightCarhood.y = " << m_rescaleVehicleZone.pRightCarhood.y << endl;

	if (m_rescaleVehicleZone.pLeftFar.y != 0 && m_rescaleVehicleZone.pRightFar.y != 0)
	{
		std::vector<cv::Point> fillContSingle;
		fillContSingle.push_back(m_rescaleVehicleZone.pLeftFar);
		fillContSingle.push_back(m_rescaleVehicleZone.pRightFar);
		fillContSingle.push_back(m_rescaleVehicleZone.pRightCarhood);
		fillContSingle.push_back(m_rescaleVehicleZone.pLeftCarhood);

		if (m_isLaneDeparture)
			cv::fillPoly(m_dsp_laneLineResult, std::vector<std::vector<cv::Point>>{fillContSingle}, cv::Scalar(0, 0, 255));
		else
			cv::fillPoly(m_dsp_laneLineResult, std::vector<std::vector<cv::Point>>{fillContSingle}, cv::Scalar(255, 0, 0));

		cv::addWeighted(m_dsp_img, 1.0, m_dsp_laneLineResult, 0.7, 0, m_dsp_img);

		cv::line(m_dsp_img, m_rescaleVehicleZone.pLeftFar, m_rescaleVehicleZone.pLeftCarhood, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
		cv::line(m_dsp_img, m_rescaleVehicleZone.pRightFar, m_rescaleVehicleZone.pRightCarhood, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

		// if (m_dsp_warningZone)
		// {
		// 	cv::line(m_dsp_img, m_rescaleRiderZone.pLeftFar, m_rescaleRiderZone.pLeftCarhood, cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
		// 	cv::line(m_dsp_img, m_rescaleRiderZone.pRightFar, m_rescaleRiderZone.pRightCarhood, cv::Scalar(0,128, 255), 2, cv::LINE_AA);

		// 	cv::line(m_dsp_img, m_rescaleHumanZone.pLeftFar, m_rescaleHumanZone.pLeftCarhood, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		// 	cv::line(m_dsp_img, m_rescaleHumanZone.pRightFar, m_rescaleHumanZone.pRightCarhood, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		// }

		// Driving Direction Vector
		if (m_isDetectLine && m_laneLineInfo.drivingDirectionVector.size() > 0)
		{
			for (int i=0; i<m_laneLineInfo.drivingDirectionVector.size(); i++)
			{
				utils::rescalePoint(
					m_laneLineInfo.drivingDirectionVector[i], m_laneLineInfo.drivingDirectionVector[i],
					m_config->modelWidth, m_config->modelHeight,
					m_videoWidth, m_videoHeight);
			}
			cv::arrowedLine(m_dsp_img, m_laneLineInfo.drivingDirectionVector[0], m_laneLineInfo.drivingDirectionVector[1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		}
	}

	if (m_dsp_vanishingLine)
		cv::line(m_dsp_img, cv::Point(0, m_yVanish), cv::Point(m_videoWidth, m_yVanish), cv::Scalar(255, 255, 255), 2);
}

void ADAS::_drawInformation()
{
	#ifdef SAV837
    int m_frameIdx = m_frameIdx_wnc;
	#endif

    cv::putText(m_dsp_imgResize, "Frame: " + std::to_string(m_frameIdx), cv::Point(10, 500),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 2, 0);

    cv::putText(m_dsp_imgResize, "Vanishing Line: " + std::to_string(m_yVanish), cv::Point(10, 520),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 2, 0);

    cv::putText(m_dsp_imgResize, "Direction: " + m_egoDirectionInfo.directionStr, cv::Point(10, 540),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 3, 0);

    cv::putText(m_dsp_imgResize, "Direction Degree: " + std::to_string(m_laneLineInfo.drivingDirectionDegree),
                cv::Point(10, 560), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 2, 0);

    cv::putText(m_dsp_imgResize, "Left Line Angle: " + std::to_string(m_currLaneInfo.leftDegree),
                cv::Point(10, 580), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 2, 0);

    cv::putText(m_dsp_imgResize, "Right Line Angle: " + std::to_string(m_currLaneInfo.rightDegree),
                cv::Point(10, 600), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 255, 0), 1, 2, 0);

	if (m_isLaneDeparture && m_dsp_laneDeparture)
	{
        cv::putText(m_dsp_imgResize, "Lane Departure Warning", cv::Point(300, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                    cv::Scalar(0, 0, 255), 2, 5, 0);
	}

	if (m_isForwardCollision && m_dsp_forwardCollision)
	{
        cv::putText(m_dsp_imgResize, "Forward Collision Warning", cv::Point(300, 100), cv::FONT_HERSHEY_COMPLEX_SMALL,
                    2, cv::Scalar(0, 0, 255), 2, 5, 0);
	}
}


void ADAS::_drawResults()
{
	int waitKey = 1;
	if (m_dsp_objectDetection)
	{
		_drawBoundingBoxes();
	}

	if (m_dsp_objectTracking)
		_drawTrackedObjects();

	if (m_dsp_laneLineDetection)
	{
		_drawLaneLines();
	}

	//TODO: turn it by setting header file only
	//TODO: develop purpose only don't want others know how we detect lane lines
	if (m_dsp_laneLineBoxes)
	{
		_drawLaneLineBoundingBoxes();
	}

	cv::resize(m_dsp_img, m_dsp_imgResize, cv::Size(1280, 720), cv::INTER_LINEAR); // Width must be mutiplication of 4

	if (m_dsp_information)
		_drawInformation();

	// m_drawResultBuffer.back().matResult = m_dsp_imgResize.clone();
	#ifndef SAV837
	if (m_dsp_results)
		cv::imshow("WNC ADAS", m_dsp_imgResize);
	#endif

	if (m_frameIdx < m_dsp_maxFrameIdx)
	{
		waitKey = 1;
	}
	else if (m_dsp_maxFrameIdx == 0)
	{
		waitKey = 1;
	}
	else if (m_isLaneDeparture && m_dbg_laneDeparture)
	{
		waitKey = 0;
	}
	else
	{
		waitKey = 0;
	}
	#ifndef SAV837
	if (m_dsp_results)
        cv::waitKey(1);
	#endif
}

void ADAS::_showADASResult()
{
	if (m_isDrawImage && m_drawResultBuffer.size() > 0)
	{
		int waitKey = 1;

		cv::Mat result = m_drawResultBuffer.back().matResult;

		#ifndef SAV837
		if (m_dsp_results)
			cv::imshow("WNC ADAS", m_dsp_imgResize);
		#endif

		if (m_frameIdx < m_dsp_maxFrameIdx)
		{
			waitKey = 1;
		}
		else if (m_dsp_maxFrameIdx == 0)
		{
			waitKey = 1;
		}
		else if (m_isLaneDeparture && m_dbg_laneDeparture)
		{
			waitKey = 0;
		}
		else
		{
			waitKey = 0;
		}
		#ifndef SAV837
		cv::waitKey(1);
		#endif
		m_drawResultBuffer.pop_front();
	}
}

// ============================================
//             Thread Management
// ============================================

bool ADAS::_runShowLogsFunc()
{
	// while ((!m_threadTerminated))
	// {
	// 	// STEP0: sleep for a while for reducing CPU loading
	// 	std::this_thread::sleep_for(std::chrono::microseconds(100));

  //   // STEP1: Start inference loop
  //   std::lock_guard<std::mutex> lock(m_mutex);

  //   if (!m_threadStarted)
  //   {
	// 	  continue;
	// 	}

	// 	if (m_inputFrameBuffer.size() > 0)
	// 	{
	// 		run(m_inputFrameBuffer.front()); //TODO:
	// 	}
	// }

	return true;
}

bool ADAS::_runDrawResultFunc()
{
	while ((!m_threadTerminated))
	{
		// STEP0: sleep for a while for reducing CPU loading
		std::this_thread::sleep_for(std::chrono::microseconds(85000)); // TODO: have to find the best value

    // STEP1: Start inference loop
    std::lock_guard<std::mutex> lock(m_mutex);

    // if (!m_threadStarted)
    // {
		//   continue;
		// }

		if (m_drawResultBuffer.size() > 0)
		{
			// Draw and Save Results
			if (m_dsp_results)
			{
				m_drawResultBuffer.back().isDraw = true;
				_drawResults();
				_showADASResult();
			}

			if (m_dsp_results && m_dbg_saveImages)
			{
				_saveDrawResults();
			}

			// Save Raw Images
			if (m_dbg_saveRawImages)
			{
				_saveRawImages();
			}
		}

		m_condition.notify_one();
	}

	return true;
}

// ============================================
//                    Utils
// ============================================
void ADAS::_updateFrameIndex()
{
  m_frameIdx = (m_frameIdx % 65535) + 1;
}
