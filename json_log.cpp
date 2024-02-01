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

JSON_LOG::JSON_LOG(std::string file)
{
	jsonFile = file;
}

JSON_LOG::~JSON_LOG()
{
}

std::string JSON_LOG::JsonLogString(ADAS_Results adasResult,
                                    ADAS_Config_S *m_config,
                                    std::vector<BoundingBox> boundingBoxLists[],
                                    std::vector<Object> m_trackedObjList,
                                    int m_frameIdx)
{
	// Create a JSON object
	json jsonData;
	json jsonDataCurrentFrame;
	if(SaveToJSONFile){
		// Read existing JSON file
		std::ifstream inFile(jsonFile);
		// json jsonData;

		if (inFile.is_open()) {
			inFile >> jsonData;
			inFile.close();
		} else {
			std::cerr << "Unable to open the file.\n";
			//return 1;
		}
	}
	// Create an "Vanisjline" array for each frame
	if(SaveVanishLineLog)
	{
		json vanishlineArray;
		json vanishline;
		vanishline["vanishlineY"] = adasResult.yVanish;
		// Add the object to the "Obj" array
		vanishlineArray.push_back(vanishline);
		jsonData["frame_ID"][std::to_string(m_frameIdx)]["vanishLineY"] = vanishlineArray;
		jsonDataCurrentFrame["frame_ID"][std::to_string(m_frameIdx)]["vanishLineY"] = vanishlineArray;
	}

	if(SaveLaneInfoLog)
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
		obj["isDetectLine"] = 		adasResult.isDetectLine;
		// Add the object to the "Obj" array
		laneArray.push_back(obj);
	
		// Add the "Obj" array to the frame
		jsonData["frame_ID"][std::to_string(m_frameIdx)]["LaneInfo"] = laneArray;
		jsonDataCurrentFrame["frame_ID"][std::to_string(m_frameIdx)]["LaneInfo"] = laneArray;
	}
	// cout<<"==========================================================="<<endl;
	// cout<<"sizeof(boundingBoxLists)="<<sizeof(boundingBoxLists)<<endl;
	// cout<<"sizeof(boundingBoxLists[0])="<<sizeof(boundingBoxLists[0])<<endl;
    if(SaveDetObjLog)
	{
		//for (int j = 0; j < sizeof(boundingBoxLists) / sizeof(boundingBoxLists[0]); j++)
		for (int j = 0; j < boundingBoxLists->size(); j++)
		//for (int j = 0; j < sizeof(boundingBoxLists); j++)
		{
			std::vector<BoundingBox>& boundingBoxList = boundingBoxLists[j];

			for (int i = 0; i < boundingBoxList.size(); i++)
			{	
				cout<<"boundingBoxList.size() = "<<boundingBoxList.size()<<endl;
				json detectArray;
				BoundingBox lastBox = boundingBoxList[i];
				BoundingBox rescaleBox(-1, -1, -1, -1, -1);  

				utils::rescaleBBox(
				lastBox, rescaleBox,
				m_config->modelWidth, m_config->modelHeight,
				m_config->frameWidth, m_config->frameHeight);

				string label = "";
				if(rescaleBox.label==0)
				{
					label = "HUMAN";
				}
				else if(rescaleBox.label==1){
					label = "RIDER";
				}
				else if(rescaleBox.label==2){
					label = "VEHICLE";
				}
				else{
					label = "UNKNOWN";
				}
				// Add track obj
				json det;
				det["detectObj.x1"] = 	 rescaleBox.x1;
				det["detectObj.y1"] = 	 rescaleBox.y1;
				det["detectObj.x2"] = 	 rescaleBox.x2;
				det["detectObj.y2"] = 	 rescaleBox.y2;
				det["detectObj.objID"] = rescaleBox.objID;
				det["detectObj.label"] = rescaleBox.label;
				det["detectObj.confidence"] = rescaleBox.confidence;
				det["detectObj.boxID"] = rescaleBox.boxID;

				// Add the track obj to the trackArray
				detectArray.push_back(det);
				// Add the "Track" array to the frame
				jsonData["frame_ID"][std::to_string(m_frameIdx)]["detectObj"][label] = detectArray;
				jsonDataCurrentFrame["frame_ID"][std::to_string(m_frameIdx)]["detectObj"][label] = detectArray;
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
			string label = "";
			if(lastBox.label==0)
			{
				label = "HUMAN";
			}
			else if(lastBox.label==1)
			{
				label = "RIDER";
			}
			else if(lastBox.label==2)
			{
				label = "VEHICLE";
			}
			// Add track obj
			json obj2;
			obj2["trackObj.x1"] = 	 rescaleBox.x1;
			obj2["trackObj.y1"] = 	 rescaleBox.y1;
			obj2["trackObj.x2"] = 	 rescaleBox.x2;
			obj2["trackObj.y2"] = 	 rescaleBox.y2;
			obj2["trackObj.status"] = trackedObj.status;
			obj2["trackObj.distanceToCamera"] = trackedObj.distanceToCamera;
			obj2["trackObj.bbox.label"] = label;
			obj2["trackedObj.id"] = trackedObj.id;

			// Add the track obj to the trackArray
			trackArray.push_back(obj2);
			// Add the "Track" array to the frame
			jsonData["frame_ID"][std::to_string(m_frameIdx)]["trackObj"][label] = trackArray;
			jsonDataCurrentFrame["frame_ID"][std::to_string(m_frameIdx)]["trackObj"][label] = trackArray;

		}
	}

    // Convert the JSON object to a string with indentation
	if(ShowJsonLog)
	{
	std::string jsonString = jsonData.dump(4);
	std::string jsonCurrentFrameString = jsonDataCurrentFrame.dump(4);
	cout<<"===================================================================================="<<endl;
	cout<<jsonCurrentFrameString<<endl;
	cout<<"===================================================================================="<<endl;
	}
    
	std::string jsonString = jsonData.dump(4);
	std::string jsonCurrentFrameString = jsonDataCurrentFrame.dump(4);
	if(SaveToJSONFile)
	{
		SaveJsonLogFile(jsonString);
	}
	return jsonCurrentFrameString;
}
void JSON_LOG::SaveJsonLogFile(std::string jsonString)
{
	// Write the updated JSON to the file
    std::ofstream outFile(jsonFile);
    if (outFile.is_open()) {
        outFile << jsonString;  // Adjust the indentation as needed
        outFile.close();
        std::cout << "Additional frame IDs appended to the JSON file.\n";
    } else {
        std::cerr << "Unable to open the file for writing.\n";
        //return 1;
    }

}
std::string JSON_LOG::GetJsonValueByKey(int targetFrameID)
{
	// Read existing JSON file
    std::ifstream inFile(jsonFile);
    json jsonData;
	std::string frameIDJsonString = "";
    if (inFile.is_open()) {
        inFile >> jsonData;
        inFile.close();
    } else {
        std::cerr << "Unable to open the file.\n";
        return "FILE_OPEN_FAILED";
    }

	// Specify the frame ID you want to retrieve
    // int targetFrameID = 2;

    // Check if the frame ID exists in the JSON
    if (jsonData["frame_ID"].contains(std::to_string(targetFrameID))) {
        // Retrieve the "Obj" array for the specified frame ID
        json objArray = jsonData["frame_ID"][std::to_string(targetFrameID)];

        // Print the values for each object in the "Obj" array
        // for (const auto& obj : objArray) {
        //     std::cout << "Vanish: " << obj["x"] << ", y: " << obj["y"]
        //               << ", w: " << obj["w"] << ", h: " << obj["h"]
        //               << ", label: " << obj["label"] << "\n";
        // }
		std::string frameIDJsonString = objArray.dump(4);
		cout<<"================ targetFrameID = "<<targetFrameID<<"======================="<<endl;
		cout<<frameIDJsonString<<endl;
		cout<<"========================================================="<<endl;
    } else {
        std::cerr << "Frame ID " << targetFrameID << " not found in the JSON.\n";
        return "FRAME_ID_NOT_FOUND_ERROR";
    }
	// std::string frameIDJsonString = objArray.dump(4);
    return frameIDJsonString;
}
std::string JSON_LOG::GetJSONFile()
{
    return jsonFile;
};
