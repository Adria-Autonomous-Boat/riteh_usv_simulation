#pragma once

#include <stdint.h>
#include <Eigen/Eigen>

static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;        // meters (m)

float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

float get_distance_to_next_waypoint_cartesian(float x_now, float y_now, float x_next, float y_next);

float get_bearing_to_next_waypoint_cartesian(float x_now, float y_now, float x_next, float y_next);

/**
 * @brief C++ class for mapping lat/lon coordinates to local coordinated using a reference position
 */
class MapProjection final
{
private:
	uint64_t _ref_timestamp{0};
	double _ref_lat{0.0};
	double _ref_lon{0.0};
	double _ref_sin_lat{0.0};
	double _ref_cos_lat{0.0};
	bool _ref_init_done{false};

public:
    /**
	* @brief Construct a new Map Projection object
	* The generated object will be uninitialized.
	* To initialize, use the `initReference` function
	*/
	MapProjection() = default;

    /**
	* Initialize the map transformation
	*
	* Initializes the transformation between the geographic coordinate system and
	* the azimuthal equidistant plane
	* @param lat in degrees (47.1234567°, not 471234567°)
	* @param lon in degrees (8.1234567°, not 81234567°)
	*/
	void initReference(double lat_0, double lon_0, uint64_t timestamp);

    /**
	 * @return true, if the map reference has been initialized before
	 */
	bool isInitialized() const { return _ref_init_done; };

	/**
	 * @return the timestamp of the reference which the map projection was initialized with
	 */
	uint64_t getProjectionReferenceTimestamp() const { return _ref_timestamp; };

    /**
     * Transform a point in the geographic coordinate system to the local
     * azimuthal equidistant plane using the projection
     * @param lat in degrees (47.1234567°, not 471234567°)
     * @param lon in degrees (8.1234567°, not 81234567°)
     * @param x north
     * @param y east
     */
    void project(double lat, double lon, float &x, float &y);

    /**
     * Transform a point in the geographic coordinate system to the local
     * azimuthal equidistant plane using the projection
     * @param lat in degrees (47.1234567°, not 471234567°)
     * @param lon in degrees (8.1234567°, not 81234567°)
     * @return the point in local coordinates as north / east
     */

    inline Eigen::Vector2f project(double lat, double lon)
    {
        Eigen::Vector2f res;
        project(lat, lon, res(0), res(1));
        return res;
    }

};
