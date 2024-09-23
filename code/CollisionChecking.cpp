///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Thomas Herring,
// Date: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // Iterate through all rectangles
    for (const auto &rect : obstacles)
    {
        // Check if the point is inside the current rectangle
        if (x >= rect.x && x <= (rect.x + rect.width) && y >= rect.y && y <= (rect.y + rect.height))
        {
            // The point is inside this rectangle, so it's not valid
            return false;
        }
    }
    // The point is outside of all rectangles
    return true;
}

// Helper function to rotate a point around the origin
void rotatePoint(double &px, double &py, double angle)
{
    double cosA = cos(angle);
    double sinA = sin(angle);
    double xNew = px * cosA - py * sinA;
    double yNew = px * sinA + py * cosA;
    px = xNew;
    py = yNew;
}

// Function to check if two line segments (p1-p2 and q1-q2) intersect
bool doLineSegmentsIntersect(double p1x, double p1y, double p2x, double p2y,
                             double q1x, double q1y, double q2x, double q2y)
{
    auto orientation = [](double ax, double ay, double bx, double by, double cx, double cy)
    {
        double val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
        if (val == 0)
            return 0;             // Collinear
        return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
    };

    auto onSegment = [](double px, double py, double qx, double qy, double rx, double ry)
    {
        return qx <= std::max(px, rx) && qx >= std::min(px, rx) &&
               qy <= std::max(py, ry) && qy >= std::min(py, ry);
    };

    int o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
    int o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
    int o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
    int o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special cases
    if (o1 == 0 && onSegment(p1x, p1y, q1x, q1y, p2x, p2y))
        return true;
    if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, p2x, p2y))
        return true;
    if (o3 == 0 && onSegment(q1x, q1y, p1x, p1y, q2x, q2y))
        return true;
    if (o4 == 0 && onSegment(q1x, q1y, p2x, p2y, q2x, q2y))
        return true;

    return false;
}

bool isValidSquare(double centerX, double centerY, double theta, double sideLength,
                   const std::vector<Rectangle> &obstacles, double hbound, double lbound)
{
    // Compute half the side length
    double halfSide = sideLength / 2.0;

    // Define the four corners of the square relative to its center
    double dx[4] = {-halfSide, halfSide, halfSide, -halfSide};
    double dy[4] = {-halfSide, -halfSide, halfSide, halfSide};

    // Rotate and translate the corners
    std::vector<std::pair<double, double>> corners(4);
    for (int i = 0; i < 4; ++i)
    {
        double px = dx[i];
        double py = dy[i];
        rotatePoint(px, py, theta);
        corners[i] = {centerX + px, centerY + py};
    }

    // Check if all corners are within the bounds of the environment
    for (const auto &corner : corners)
    {
        if (corner.first < lbound || corner.first > hbound ||
            corner.second < lbound || corner.second > hbound)
        {
            return false;
        }
    }

    // Check if any square edge intersects with any rectangle edge
    for (const auto &rect : obstacles)
    {
        // Define the four corners of the rectangle
        std::pair<double, double> rectCorners[4] = {
            {rect.x, rect.y},
            {rect.x + rect.width, rect.y},
            {rect.x + rect.width, rect.y + rect.height},
            {rect.x, rect.y + rect.height}};

        // Define the four edges of the rectangle
        std::pair<int, int> rectEdges[4] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}};

        // Check each edge of the square
        for (int i = 0; i < 4; ++i)
        {
            double p1x = corners[i].first;
            double p1y = corners[i].second;
            double p2x = corners[(i + 1) % 4].first;
            double p2y = corners[(i + 1) % 4].second;

            // Check against each edge of the rectangle
            for (const auto &edge : rectEdges)
            {
                double q1x = rectCorners[edge.first].first;
                double q1y = rectCorners[edge.first].second;
                double q2x = rectCorners[edge.second].first;
                double q2y = rectCorners[edge.second].second;

                if (doLineSegmentsIntersect(p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y))
                {
                    return false;
                }
            }
        }
    }

    // The square does not intersect any rectangle and is within bounds
    return true;
}