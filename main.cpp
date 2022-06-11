#include "Triangle.hpp"
#include "rasterizer.hpp"
#include "__INCLUDE__.h"
#include "Triangle.hpp"
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float sin_theta = sin(rotation_angle);
    float cos_theta = cos(rotation_angle);
    model(0,0) = cos_theta;
    model(1,1) = cos_theta;
    model(0,1) = -sin_theta;
    model(1,0) = sin_theta;

    return model;
}
Eigen::Matrix4f get_rodrigues_model_matrix(float rotation_angle, Vector3f &axis)
{
    // axis is the cow vector pass zero point
    // Create the model matrix for rotating the triangle around the any axis.
    // Then return it.
    float cos_theta = cos(rotation_angle);
    float sin_theta = sin(rotation_angle);
    Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity() * cos_theta;
    float x2 = axis[0]*axis[0];
    float xy = axis[0]*axis[1];
    float xz = axis[0]*axis[2];
    float y2 = axis[1]*axis[1];
    float yz = axis[1]*axis[2];
    float z2 = axis[2]*axis[2];
    temp(0,0) = x2;
    temp(1,1) = y2;
    temp(2,2) = z2;
    temp(0,1) = temp(1,0) = xy;
    temp(0,2) = temp(2,0) = xz;
    temp(1,2) = temp(2,1) = yz;
    model += (1-cos_theta) * temp;
    temp(0,0) = temp(1,1) = temp(2,2) = 0;
    temp(0,1) = -axis[2];
    temp(1,0) = axis[2];
    temp(0,2) = axis[1];
    temp(2,0) = -axis[1];
    temp(1,2) = -axis[0];
    temp(2,1) = axis[0];
    model += sin_theta * temp;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float tan_theta = tan(0.5 * eye_fov);
    float far_near_len = zNear - zFar;
    projection << 1 / (aspect_ratio * tan_theta),0 ,0 ,0,
                  0, 1 / tan_theta, 0, 0,
                  0, 0, (zNear+zFar) / far_near_len, -2 * zFar * zNear / far_near_len,
                  0, 0, 1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
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
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
