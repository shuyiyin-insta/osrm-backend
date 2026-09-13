@routing @isochrone @testbot
Feature: Isochrone service

    Background:
        Given the profile "testbot"
        And a grid size of 100 meters

    Scenario: Return one ordered GeoJSON polygon feature for each duration contour in seconds
        Given the node map
            """
            a b c
            d e f
            """
        And the ways
            | nodes |
            | abc   |
            | def   |
            | ad    |
            | be    |
            | cf    |

        When I request an isochrone from "e" with contours_seconds "20,10"
        Then the isochrone response should be a GeoJSON FeatureCollection with "2" "MultiPolygon" features
        And the isochrone contour_seconds properties should be "20,10"
        And the isochrone response should have "1" waypoint

    Scenario: Support line output and zero-valued geometry controls as exact no-ops
        Given the node map
            """
            a b c
            """
        And the ways
            | nodes |
            | abc   |

        When I request an isochrone from "b" with contours_seconds "20" and polygons "false"
        Then the isochrone response should be a GeoJSON FeatureCollection with "1" "MultiLineString" features

        When I request equivalent raw and zero-processed isochrones from "b" with contours_seconds "20"
        Then the isochrone response should be a GeoJSON FeatureCollection with "1" "MultiPolygon" features
        And the raw and zero-processed isochrone responses should be identical

    Scenario: Respect directed network travel for inbound and outbound contours
        Given a grid size of 500 meters
        And the node map
            """
            a 1 b
            d   c
            """
        And the ways
            | nodes | oneway |
            | ab    | yes    |
            | bc    | yes    |
            | cd    | yes    |
            | da    | yes    |

        When I request inbound and outbound isochrones from "1" with contours_seconds "75"
        Then the inbound and outbound isochrone geometries should differ

    Scenario: Materialize the precise cutoff partway along a traversed road
        Given the node locations
            | node | lon  | lat |
            | a    | 1    | 1   |
            | b    | 1.02 | 1   |
        And the ways
            | nodes | duration |
            | ab    | 20       |

        When I request an isochrone from "a" with contours_seconds "10"
        Then the isochrone response should be a GeoJSON FeatureCollection with "1" "MultiPolygon" features
        And the isochrone boundary should stop partway from "a" to "b"

    Scenario: Use the duration of the profile-weight-optimal path
        Given a grid size of 500 meters
        And the profile file
            """
            local functions = require('testbot')
            functions.setup_testbot = functions.setup

            functions.setup = function()
              local profile = functions.setup_testbot()
              profile.properties.weight_name = 'steps'
              profile.properties.traffic_signal_penalty = 0
              profile.properties.u_turn_penalty = 0
              return profile
            end

            functions.process_way = function(profile, way, result)
              result.forward_mode = mode.driving
              result.backward_mode = mode.driving
              result.duration = tonumber(way:get_value_by_key('duration'))
              result.weight = tonumber(way:get_value_by_key('weight'))
            end

            return functions
            """
        And the node map
            """
            a x
             y m t
            """
        And the ways
            | nodes | duration | weight |
            | axm   | 5        | 100    |
            | aym   | 15       | 1      |
            | mt    | 5        | 1      |

        # The faster route via x has weight 101. Route/table and isochrone choose
        # the lower-weight route via y, whose selected duration reaches t only at 20s.
        When I request an isochrone from "a" with contours_seconds "10"
        Then the isochrone response should be a GeoJSON FeatureCollection with "1" "MultiPolygon" features
        And the isochrone should contain "x" and not contain "t"

    Scenario: Respect excluded road classes
        Given a grid size of 500 meters
        And the query options
            | exclude | motorway |
        And the node map
            """
            a b c
            """
        And the ways
            | nodes | highway  |
            | ab    | primary  |
            | bc    | motorway |

        When I request an isochrone from "a" with contours_seconds "200"
        Then the isochrone response should be a GeoJSON FeatureCollection with "1" "MultiPolygon" features
        And the isochrone should contain "b" and not contain "c"

    Scenario: Reject missing, malformed, and out-of-range duration contour requests
        Given the node map
            """
            a b
            """
        And the ways
            | nodes |
            | ab    |

        When I request /isochrone/v1/testbot/1,1
        Then the isochrone HTTP status should be 400
        And status code should be InvalidOptions

        When I request /isochrone/v1/testbot/1,1?contours_seconds=0
        Then the isochrone HTTP status should be 400
        And status code should be InvalidOptions

        When I request /isochrone/v1/testbot/1,1?contours_seconds=10&direction=sideways
        Then the isochrone HTTP status should be 400
        And status code should be InvalidQuery

        When I request /isochrone/v1/testbot/1,1?contours_seconds=901
        Then the isochrone HTTP status should be 400
        And status code should be InvalidOptions
