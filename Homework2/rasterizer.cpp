// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Eigen::Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    auto check = [](Eigen::Vector3f &vec1, Eigen::Vector3f &vec2) -> bool
    {
        return vec1.cross(vec2)[2] < 0;
    };
    Eigen::Vector3f p = {x + 0.5f, y + 0.5f, _v[0][2]};
    Eigen::Vector3f v0v1 = _v[1] - _v[0], v1v2 = _v[2] - _v[1], v2v0 = _v[0] - _v[2];
    Eigen::Vector3f v0p = _v[0] - p, v1p = _v[1] - p, v2p = _v[2] - p;
    return check(v0v1, v0p) && check(v1v2, v1p) && check(v2v0, v2p);
}

static bool insideTriangle(float x, float y, const Eigen::Vector3f* _v, int)
{   
    // TODO : Implement this function to calculate how many points in (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    auto check = [](Eigen::Vector3f &vec1, Eigen::Vector3f &vec2) -> bool
    {
        return vec1.cross(vec2)[2] < 0;
    };

    int ret = 0;

    Eigen::Vector3f p = {x + 0.25f, y + 0.25f, _v[0][2]};
    Eigen::Vector3f v0v1 = _v[1] - _v[0], v1v2 = _v[2] - _v[1], v2v0 = _v[0] - _v[2];
    Eigen::Vector3f v0p = _v[0] - p, v1p = _v[1] - p, v2p = _v[2] - p;
    
    return check(v0v1, v0p) && check(v1v2, v1p) && check(v2v0, v2p);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;

    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //Homogeneous division
        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type, int)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;

    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //Homogeneous division
        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
            // t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t, 2);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4(); // The 3 vertices of the triangle t (Vector4f)

    std::array<Eigen::Vector3f, 3> v3; // The 3 vertices of the triangle t (Vector3f)

    for (size_t i = 0; i < 3; ++i)
    {
        Eigen::Vector4f &temp = v[i];
        v3[i] = {temp.x() / temp.w(), temp.y() / temp.w(), temp.z() / temp.w()};
    }

    auto min = [](float a, float b) -> float
    {
        return a < b ? a : b;
    };

    auto max = [](float a, float b) -> float
    {
        return a < b ? b : a;
    };
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle 

    int left = min(min(v3[0][0], v3[1][0]), v3[2][0]);
    int right = max(max(v3[0][0], v3[1][0]), v3[2][0]);
    int bottom = min(min(v3[0][1], v3[1][1]), v3[2][1]);
    int top = max(max(v3[0][1], v3[1][1]), v3[2][1]);

    for (int i = left; i < right; ++i)
    {
        for (int j = bottom; j < top; ++j)
        {
            if (insideTriangle(i, j, v3.data()))
            {
                // If so, use the following code to get the interpolated z value.
                auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if (depth_buf[get_index(i, j)] > z_interpolated)
                {
                    depth_buf[get_index(i, j)] = z_interpolated;
                    set_pixel({i + 0.0f, j + 0.0f, 0.0f}, t.getColor());
                }
            }
        }
    }
}

//Screen space rasterization with supersampling
void rst::rasterizer::rasterize_triangle(const Triangle& t, int)
{
    auto v = t.toVector4(); // The 3 vertices of the triangle t (Vector4f)

    std::array<Eigen::Vector3f, 3> v3; // The 3 vertices of the triangle t (Vector3f)

    for (size_t i = 0; i < 3; ++i)
    {
        Eigen::Vector4f &temp = v[i];
        v3[i] = {temp.x() / temp.w(), temp.y() / temp.w(), temp.z() / temp.w()};
    }

    auto min = [](float a, float b) -> float
    {
        return a < b ? a : b;
    };

    auto max = [](float a, float b) -> float
    {
        return a < b ? b : a;
    };
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle 

    int left = min(min(v3[0][0], v3[1][0]), v3[2][0]);
    int right = max(max(v3[0][0], v3[1][0]), v3[2][0]);
    int bottom = min(min(v3[0][1], v3[1][1]), v3[2][1]);
    int top = max(max(v3[0][1], v3[1][1]), v3[2][1]);

    float d = 0.25f;
    int N2 = 4;
    float sumZ = 0.0f;

    for (int i = left; i < right; ++i)
    {
        for (int j = bottom; j < top; ++j)
        {
            for (int iIn = 0; iIn < 2; ++iIn)
            {
                for (int jIn = 0; jIn < 2; ++jIn)
                {
                    if (insideTriangle(i + iIn * 0.5f, j + jIn * 0.5f, v3.data(), 0))
                    {
                        auto [alpha, beta, gamma] = computeBarycentric2D(i + iIn * 0.5f, j + jIn * 0.5f, t.v);
                        float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        if (depth_buf_ss[get_index(i, j)][iIn * 2 + jIn] > z_interpolated)
                        {
                            depth_buf_ss[get_index(i, j)][iIn * 2 + jIn] = z_interpolated;
                            frame_buf_ss[get_index(i, j)][iIn * 2 + jIn] = t.getColor();
                        }
                    }
                }
            }
            set_pixel({i + 0.0f, j + 0.0f, 0.0f}, (frame_buf_ss[get_index(i, j)][0] + frame_buf_ss[get_index(i, j)][1] +
            frame_buf_ss[get_index(i, j)][2] + frame_buf_ss[get_index(i, j)][3]) / 4.0f);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        // std::numeric_limits<float>::infinity() is positive
        float inf = std::numeric_limits<float>::infinity();
        std::fill(depth_buf.begin(), depth_buf.end(), inf);
        std::fill(depth_buf_ss.begin(), depth_buf_ss.end(), std::array<float, 4>{inf, inf, inf, inf});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    frame_buf_ss.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_ss.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return x + y * width;
    // old: return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = point.x() + point.y() * width;
    // old: auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}

// clang-format on