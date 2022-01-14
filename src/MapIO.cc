#include "MapIO.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <fstream>

namespace ORB_SLAM2 {

MapIO::MapIO(Data::Map* map, Data::KeyFrameDatabase* keyframeDb, Data::ORBVocabulary* orbVocab)
    :  map_(map), keyframeDb_(keyframeDb), orbVocab_(orbVocab) {}

void MapIO::SaveMap(const std::string& path) {
    std::lock_guard<std::mutex> lock(map_->mMutexMapUpdate);

    assert(map_);
    nlohmann::json keyfrms;
    nlohmann::json landmarks;
    map_->ToJson(keyfrms, landmarks);

    nlohmann::json json{{"keyframes", keyfrms},
                        {"landmarks", landmarks},
                        {"frame_next_id", static_cast<unsigned int>(Frame::nNextId)},
                        {"keyframe_next_id", static_cast<unsigned int>(KeyFrame::nNextId)},
                        {"landmark_next_id", static_cast<unsigned int>(MapPoint::nNextId)}};

    std::ofstream ofs(path, std::ios::out | std::ios::binary);

    if (ofs.is_open()) {
        cout << "save the MessagePack file of database to " << path << '\n';
        const auto msgpack = nlohmann::json::to_msgpack(json);
        ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
        ofs.close();
    }
    else {
        cerr << "cannot create a file at " << path << '\n';
    }
}

void MapIO::LoadMap(const std::string& path) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. initialize database

    assert(cam_db_ && map_db_ && bow_db_ && bow_vocab_);
    map_db_->clear();
    bow_db_->clear();

    // 2. load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        spdlog::critical("cannot load the file at {}", path);
        throw std::runtime_error("cannot load the file at " + path);
    }

    spdlog::info("load the MessagePack file of database from {}", path);
    std::vector<uint8_t> msgpack;
    while (true) {
        uint8_t buffer;
        ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
        if (ifs.eof()) {
            break;
        }
        msgpack.push_back(buffer);
    }
    ifs.close();

    // 3. parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // 4. load database

    // load static variables
    data::frame::next_id_ = json.at("frame_next_id").get<unsigned int>();
    data::keyframe::next_id_ = json.at("keyframe_next_id").get<unsigned int>();
    data::landmark::next_id_ = json.at("landmark_next_id").get<unsigned int>();
    // load database
    const auto json_cameras = json.at("cameras");
    cam_db_->from_json(json_cameras);
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    map_db_->from_json(cam_db_, bow_vocab_, bow_db_, json_keyfrms, json_landmarks);
    const auto keyfrms = map_db_->get_all_keyframes();
    for (const auto keyfrm : keyfrms) {
        bow_db_->add_keyframe(keyfrm);
    }
}

} // namespace ORB_SLAM2
