#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float sinVal, cosVal;
    sincosf32(rotation_angle / 180.0 * MY_PI, &sinVal, &cosVal);

    model <<
        cosVal, -sinVal, 0.0f, 0.0f,
        sinVal, cosVal, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;

    return model;
}

// abs(axis) should be 1
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float rotation_angle)
{
    float sinVal, cosVal;
    sincosf32(rotation_angle / 180.0 * MY_PI, &sinVal, &cosVal);

    Eigen::Matrix3f rotation3 = cosVal * Eigen::Matrix3f::Identity();

    Eigen::Matrix3f N;
    N <<
        0.0f, -axis[2], axis[1],
        axis[2], 0.0f, -axis[0],
        -axis[1], axis[0], 0.0f;

    rotation3 += (1 - cosVal) * axis * axis.transpose() + sinVal * N;

    Eigen::Matrix4f rotation;
    rotation <<
        rotation3(0, 0), rotation3(0, 1), rotation3(0, 2), 0.0f,
        rotation3(1, 0), rotation3(1, 1), rotation3(1, 2), 0.0f,
        rotation3(2, 0), rotation3(2, 1), rotation3(2, 2), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    
    return rotation;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float t = tanf32(eye_fov / 180.0 * MY_PI / 2.0) * abs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;
    Eigen::Matrix4f perspToOrtho;
    perspToOrtho <<
        zNear, 0.0f, 0.0f, 0.0f,
        0.0f, zNear, 0.0f, 0.0f,
        0.0f, 0.0f, zNear + zFar, -zNear * zFar,
        0.0f, 0.0f, 1.0f, 0.0f;

    Eigen::Index a;
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = 2.0f / (r - l);
    scale(1, 1) = 2.0f / (t - b);
    scale(2, 2) = 2.0f / (zNear - zFar); 

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0, 3) = -(r + l) / 2.0f;
    translate(1, 3) = -(t + b) / 2.0f;
    translate(2, 3) = -(zNear + zFar) / 2.0f;

    projection *= (scale * translate * perspToOrtho);

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(90.0f, 1.0f, -1.0f, -10.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation({1, 0, 0}, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, -1.0f, -10.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 7;
        }
        else if (key == 'd') {
            angle -= 7;
        }
    }

    return 0;
}
