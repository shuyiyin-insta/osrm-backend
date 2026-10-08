@routed @options @isochrone
Feature: osrm-routed isochrone search-node budget options

    Background:
        Given the profile "testbot"
        And the node map
            """
            a b
            """
        And the ways
            | nodes |
            | ab    |
        And the data has been extracted
        And the data has been contracted

    Scenario Outline: Accept a positive or unlimited search-node budget
        When I run "osrm-routed {processed_file} --algorithm CH --trial=true --max-isochrone-search-nodes <limit>"
        Then it should exit successfully

        Examples:
            | limit      |
            | -1         |
            | 1          |
            | 2147483647 |

    Scenario Outline: Reject invalid search-node budgets
        When I try to run "osrm-routed {processed_file} --algorithm CH --trial=true --max-isochrone-search-nodes <limit>"
        Then it should exit with an error

        Examples:
            | limit      |
            | -2         |
            | 0          |
            | 1.5        |
            | NaN        |
            | Infinity   |
            | 2147483648 |
