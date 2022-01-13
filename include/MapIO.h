#ifndef MAP_IO_H
#define MAP_IO_H

#include "Map.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include <string>

namespace ORB_SLAM2 {

class MapIO {
public:
    /**
     * Constructor
     */
    MapIO(Map* map, KeyFrameDatabase* keyframeDb, ORBVocabulary* orbVocab);

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
    Map* const map_ = nullptr;
    //! BoW database
    KeyFrameDatabase* const keyframeDb_ = nullptr;
    //! BoW vocabulary
    ORBVocabulary* const orbVocab_ = nullptr;
};

} // namespace ORB_SLAM2

#endif // MAP_IO_H
