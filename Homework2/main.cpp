// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

/**
 * @brief Get a 4x4 view matrix
 * @param eye_pos The position of the eye or camara
 * @return View matrix
 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate * view;

    return view;
}

/**
 * @brief Get a 4x4 model matrix (For we needn't to rotate the model, it returns the identity matrix)
 * @param rotation_angle The angle (unit: degree) for rotating counterclockwise along the z axis, which should be in the range [0.0f, 360.0f)
 * @return View matrix
 */
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

/**
 * @brief Get a 4x4 projection matrix.
 * @param eye_fov Vertical field-of-view (unit: degree), which should be in the range [0.0f, 360.0f)
 * @param aspect_ratio The ratio of width to height
 * @param zNear The z coordinate of the near clip plane
 * @param zFar The z coordinate of the far clip plane
 * @return View matrix
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float t = tanf32(static_cast<float>(MY_PI) * eye_fov / 360.0f) * abs(zNear);
    float r = aspect_ratio * t;

    Eigen::Matrix4f perspToOrtho;
    perspToOrtho <<
        zNear, 0.0f, 0.0f, 0.0f,
        0.0f, zNear, 0.0f, 0.0f,
        0.0f, 0.0f, zNear + zFar, -zNear * zFar,
        0.0f, 0.0f, 1.0f, 0.0f;

    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = 1.0 / r;
    scale(1, 1) = 1.0 / t;
    scale(2, 2) = 2.0f / (zNear - zFar); 

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = 0.0f;
    translate(1, 3) = 0.0f;
    translate(2, 3) = -(zNear + zFar) / 2.0f;

    projection *= (scale * translate * perspToOrtho);

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0; // The angle (unit: degree) for rotating counterclockwise along the z axis

    bool command_line = false; // If it is false, a window will be opened to display the image

    std::string filename = "output.png"; // Output filename

    // The program is run with a command line parameter
    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    // The size of the window is 700 x 700
    rst::rasterizer r(700, 700);

    // The position of the eye
    Eigen::Vector3f eye_pos = {0.0f,0.0f,5.0f};

    // The vertices of the two triangles
    std::vector<Eigen::Vector3f> pos = {
        {2, 0, -2},
        {0, 2, -2},
        {-2, 0, -2},
        {3.5, -1, -5},
        {2.5, 1.5, -5},
        {-1, 0.5, -5}
    };

    std::vector<Eigen::Vector3i> ind = {
        {0, 1, 2},
        {3, 4, 5}
    };

    // The colors of the vertices of the two triangles
    std::vector<Eigen::Vector3f> cols = {
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {217.0, 238.0, 185.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0},
        {185.0, 217.0, 238.0}
    };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    // super-sampling version
    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle, 0);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on