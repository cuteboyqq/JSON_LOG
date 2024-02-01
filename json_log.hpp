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

#ifndef __JSON_LOG__
#define __JSON_LOG__

#include "json.hpp"
#include <iostream>
#include <fstream>

#include <math.h>  //sin
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <assert.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "point.hpp"
#include "bounding_box.hpp"
#include "dataStructures.h"
#include "adas.hpp"
#include "bounding_box.hpp"
using namespace std;

class JSON_LOG
{
public:
	JSON_LOG(std::string file);
	~JSON_LOG();
	// Return LOG String with JSON format
	std::string JsonLogString(ADAS_Results adasResult,
							  ADAS_Config_S* m_config,
							  std::vector<BoundingBox> boundingBoxLists[],
							  std::vector<Object> m_trackedObjList,
							  int m_frameIdx);


	std::string JsonLogString_2(ADAS_Results adasResult,
							  ADAS_Config_S* m_config,
							  std::vector<BoundingBox> m_humanBBoxList,
							  std::vector<BoundingBox> m_riderBBoxList,
							  std::vector<BoundingBox> m_vehicleBBoxList,
							  std::vector<BoundingBox> m_roadSignBBoxList,
							  std::vector<BoundingBox> m_stopSignBBoxList,
							  std::vector<Object> m_trackedObjList,
							  int m_frameIdx);

	void SaveJsonLogFile(std::string jsonString);

	std::string GetJsonValueByKey(int targetFrameID);

	std::string GetJSONFile();

private:
	//Show log on terminal
	bool ShowJsonLog = true;

	//Enable save log type
	bool SaveTrackObjLog = true;
	bool SaveLaneInfoLog = true;
	bool SaveDetObjLog = true;
	bool SaveVanishLineLog = true;
	bool SaveToJSONFile = false;
	bool SaveLDWLog = true;
	bool SaveFCWLog = true;
	
	std::string jsonString;
	std::string jsonFile;
};

#endif