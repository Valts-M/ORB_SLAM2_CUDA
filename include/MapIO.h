#ifndef MAP_IO_H
#define MAP_IO_H

#include "Map.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include <string>

namespace ORB_SLAM2 {

namespace Data {
class KeyFrameDatabase;
class Map;
class ORBVocabulary;
} // namespace data


class MapIO {
public:
    /**
     * Constructor
     */
    MapIO(Data::Map* map, Data::KeyFrameDatabase* keyframeDb, Data::ORBVocabulary* orbVocab);

    /**
     * Destructor
     */
    ~MapIO() = default;

    /**
     * Save the map database as MessagePack
     */
    void SaveMap(const std::string& path);

    /**
     * Load the map database from MessagePack
     */
    void LoadMap(const std::string& path);

private:

    //! map_database
    Data::Map* const map_ = nullptr;
    //! BoW database
    Data::KeyFrameDatabase* const keyframeDb_ = nullptr;
    //! BoW vocabulary
    Data::ORBVocabulary* const orbVocab_ = nullptr;
};

} // namespace ORB_SLAM2

#endif // MAP_IO_H
