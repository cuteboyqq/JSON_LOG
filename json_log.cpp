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

#include "json_log.hpp"
#include "dms.hpp"
using json = nlohmann::json;

std::string JSON_LOG::JsonLogString(ADAS_Results adasResult,
									ADAS_Config_S* m_config,
									std::vector<BoundingBox> boundingBoxLists[],
									std::vector<Object> m_trackedObjList,
									int m_frameIdx)
{
	// Create a JSON object
    json jsonData;
	
	// Create an "Obj" array for each frame
	

	if(SaveLaneInfo)
	{	
		json laneArray;
		// Add lane info
		json obj;
		obj["pLeftFar.x"] = 		adasResult.pLeftFar.x;
		obj["pLeftFar.y"] = 		adasResult.pLeftFar.y;
		obj["pLeftCarhood.x"] = 	adasResult.pLeftCarhood.x;
		obj["pLeftCarhood.y"] = 	adasResult.pLeftCarhood.y;
		obj["pRightFar.x"] = 		adasResult.pRightFar.x;
		obj["pRightFar.y"] = 		adasResult.pRightFar.y;
		obj["pRightCarhood.x"] = 	adasResult.pRightCarhood.x;
		obj["pRightCarhood.y"] = 	adasResult.pRightCarhood.y;
		
		// Add the object to the "Obj" array
		laneArray.push_back(obj);
	
		// Add the "Obj" array to the frame
		jsonData["frame_ID"][std::to_string(m_frameIdx)]["LaneInfo"] = laneArray;
	}
	
    if(SaveDetObjLog)
	{
		for (int j = 0; j < sizeof(boundingBoxLists) / sizeof(boundingBoxLists[0]); j++)
		{
			std::vector<BoundingBox>& boundingBoxList = boundingBoxLists[j];

			for (int i = 0; i < boundingBoxList.size(); i++)
			{
				json detectArray;
				BoundingBox lastBox = boundingBoxList[i];
				BoundingBox rescaleBox(-1, -1, -1, -1, -1);

				utils::rescaleBBox(
				lastBox, rescaleBox,
				m_config->modelWidth, m_config->modelHeight,
				m_config->frameWidth, m_config->frameHeight);

				// Add track obj
				json det;
				det["detectObj.x1"] = 	 rescaleBox.x1;
				det["detectObj.y1"] = 	 rescaleBox.y1;
				det["detectObj.x2"] = 	 rescaleBox.x2;
				det["detectObj.y2"] = 	 rescaleBox.y2;
				det["detectObj.label"] = rescaleBox.label;

				// Add the track obj to the trackArray
				detectArray.push_back(det);
				// Add the "Track" array to the frame
				jsonData["frame_ID"][std::to_string(m_frameIdx)]["detectObj"][std::to_string(i)] = detectArray;
			}
		}
	}
	
	if(SaveTrackObjLog)
	{
		for (int i = 0; i < m_trackedObjList.size(); i++)
		{   // Create an "Track" array for each frame	
			json trackArray;
			const Object& trackedObj = m_trackedObjList[i];
			if (trackedObj.bboxList.empty())
				continue;
			BoundingBox lastBox = trackedObj.bboxList.back();
			BoundingBox rescaleBox(-1, -1, -1, -1, -1);

			// Rescale BBoxes
			//#ifdef SAV837
			utils::rescaleBBox(
			lastBox, rescaleBox,
			m_config->modelWidth, m_config->modelHeight,
			m_config->frameWidth, m_config->frameHeight);
			//#endif
			// Add track obj
			json obj2;
			obj2["trackObj.x1"] = 	 rescaleBox.x1;
			obj2["trackObj.y1"] = 	 rescaleBox.y1;
			obj2["trackObj.x2"] = 	 rescaleBox.x2;
			obj2["trackObj.y2"] = 	 rescaleBox.y2;
			obj2["trackObj.status"] = trackedObj.status;
			obj2["trackObj.distanceToCamera"] = trackedObj.distanceToCamera;
			obj2["trackObj.bbox.label"] = trackedObj.bbox.label;

			// Add the track obj to the trackArray
			trackArray.push_back(obj2);
			// Add the "Track" array to the frame
			jsonData["frame_ID"][std::to_string(m_frameIdx)]["trackObj"][std::to_string(trackedObj.id)] = trackArray;

		}
	}

    // Convert the JSON object to a string with indentation
	if(ShowJsonLog)
	{
	std::string jsonString = jsonData.dump(4);
	cout<<"===================================================================================="<<endl;
	cout<<jsonString<<endl;
	cout<<"===================================================================================="<<endl;
	}
    
	std::string jsonString = jsonData.dump(4);
	return jsonString;
};

