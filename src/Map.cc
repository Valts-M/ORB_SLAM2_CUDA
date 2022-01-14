/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::Save ( const string& filename )
{
    cerr<<"Map Saving to "<<filename <<endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);
    cerr << "The number of MapPoints is :"<<mspMapPoints.size()<<endl;

    //Number of map points
    unsigned long int nMapPoints = mspMapPoints.size();
    f.write((char*)&nMapPoints, sizeof(nMapPoints) );
    //Save MapPoints in turn
    for ( auto mp: mspMapPoints )
        SaveMapPoint( f, mp );

    //Get the index value of each MapPoints, that is, start counting from 0, initialize mmpnMapPointsIdx  
    GetMapPointsIdx(); 

    cerr <<"The number of KeyFrames:"<<mspKeyFrames.size()<<endl;
    //Number of key frames
    unsigned long int nKeyFrames = mspKeyFrames.size();
    f.write((char*)&nKeyFrames, sizeof(nKeyFrames));

    //Save the key frames KeyFrames in turn
    for ( auto kf: mspKeyFrames )
        SaveKeyFrame( f, kf );

    for (auto kf:mspKeyFrames )
    {
        //Get the parent node of the current key frame and save the ID of the parent node
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if ( parent )
            parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));
        //Get the size of the associated key frame of the current key frame, and save the ID and weight of each associated key frame in turn;
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con, sizeof(nb_con));
        for ( auto ckf: kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    f.close();
    cerr<<"Map Saving Finished!"<<endl;
}

void Map::SaveMapPoint( ofstream& f, MapPoint* mp)
{   
     //Save the current MapPoint ID and world coordinate values
     f.write((char*)&mp->mnId, sizeof(mp->mnId));
     cv::Mat mpWorldPos = mp->GetWorldPos();
     f.write((char*)& mpWorldPos.at<float>(0),sizeof(float));
     f.write((char*)& mpWorldPos.at<float>(1),sizeof(float));
     f.write((char*)& mpWorldPos.at<float>(2),sizeof(float));
}

void Map::SaveKeyFrame( ofstream &f, KeyFrame* kf )
{
//Save the ID and timestamp of the current key frame
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
    //Save the pose matrix of the current key frame
    cv::Mat Tcw = kf->GetPose();
    //Save rotation matrix by quaternion
    std::vector<float> Quat = Converter::toQuaternion(Tcw);
    for ( int i = 0; i < 4; i ++ )
        f.write((char*)&Quat[i],sizeof(float));
    //Save the translation matrix
    for ( int i = 0; i < 3; i ++ )
        f.write((char*)&Tcw.at<float>(i,3),sizeof(float));


    //Save the rotation matrix directly
    //  for ( int i = 0; i < Tcw.rows; i ++ )
    //  {
    //      for ( int j = 0; j < Tcw.cols; j ++ )
    //      {
    //              f.write((char*)&Tcw.at<float>(i,j), sizeof(float));
    //              //cerr<<"Tcw.at<float>("<<i<<","<<j<<"):"<<Tcw.at<float>(i,j)<<endl;
    //      }
    //    }

    //Save the number of ORB features contained in the current key frame
    //cerr<<"kf->N:"<<kf->N<<endl;
    f.write((char*)&kf->N, sizeof(kf->N));
    //Save each ORB feature point
    for( int i = 0; i < kf->N; i ++ )
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle,sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        //Save the descriptor of the current feature point
        for (int j = 0; j < kf->mDescriptors.cols; j ++ )
                f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        //Save the index value of MapPoints corresponding to the current ORB feature
        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL  )
                mnIdx = ULONG_MAX;
        else
                mnIdx = mmpnMapPointsIdx[mp];

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }
}

void Map::GetMapPointsIdx()
{
        unique_lock<mutex> lock(mMutexMap);
        unsigned long int i = 0;
        for ( auto mp: mspMapPoints )
        {
                mmpnMapPointsIdx[mp] = i;
                i += 1;
        }
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
