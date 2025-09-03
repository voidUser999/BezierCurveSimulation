/******************************************************************************
 *                                                                            *
 *  Copyright (c) 2023 Ojaswa Sharma. All rights reserved.                    *
 *                                                                            *
 *  Author: Ojaswa Sharma                                                     *
 *  E-mail: ojaswa@iiitd.ac.in                                                *
 *                                                                            *
 *  This code is provided solely for the purpose of the CSE 333/533 course    *
 *  at IIIT Delhi. Unauthorized reproduction, distribution, or disclosure     *
 *  of this code, in whole or in part, without the prior written consent of   *
 *  the author is strictly prohibited.                                        *
 *                                                                            *
 *  This code is provided "as is", without warranty of any kind, express      *
 *  or implied, including but not limited to the warranties of                *
 *  merchantability, fitness for a particular purpose, and noninfringement.   *
 *                                                                            *
 ******************************************************************************/

#include "utils.h"

#define DRAW_PIECEWISE_BEZIER 1 // Use to switch between drawing control polyline and piecewise bezier curves
#define SAMPLES_PER_BEZIER 10   // Sample each Bezier curve as N=10 segments and draw as connected lines

// GLobal variables
std::vector<float> controlPoints;
std::vector<float> controlPolyline;
std::vector<float> piecewiseBezier;
std::vector<float> tangentLines;
int width = 640, height = 640;
bool controlPointsUpdated = false;
bool controlPointsFinished = false;
int selectedControlPoint = -1;
bool showTangents = true;

unsigned int VBO_tangentLines, VAO_tangentLines;
struct point2d
{
    float x, y;
};

void calculateControlPolyline()
{
    // Since controlPolyline is just a polyline, we can simply copy the control points and plot.
    // However to show how a piecewise parametric curve needs to be plotted, we sample t and
    // evaluate it as a piecewise linear bezier curve.
    // controlPolyline.assign(controlPoints.begin(), controlPoints.end());

    controlPolyline.clear();
    int sz = controlPoints.size(); // Contains 3 points/vertex. Ignore Z
    float x[2], y[2];
    float delta_t = 1.0 / (SAMPLES_PER_BEZIER - 1.0);
    float t;
    for (int i = 0; i < (sz - 3); i += 3)
    {
        x[0] = controlPoints[i];
        y[0] = controlPoints[i + 1];
        x[1] = controlPoints[i + 3];
        y[1] = controlPoints[i + 4];
        controlPolyline.push_back(x[0]);
        controlPolyline.push_back(y[0]);
        controlPolyline.push_back(0.0);

                t = 0.0;
        for (float j = 1; j < (SAMPLES_PER_BEZIER - 1); j++)
        {
            t += delta_t;
            controlPolyline.push_back(x[0] + t * (x[1] - x[0]));
            controlPolyline.push_back(y[0] + t * (y[1] - y[0]));
            controlPolyline.push_back(0.0);
        }
        // No need to add the last point for this segment, since it will be added as first point in next.
    }
    // However, add last point of the polyline here (i.e, the last control point)
    controlPolyline.push_back(x[1]);
    controlPolyline.push_back(y[1]);
    controlPolyline.push_back(0.0);
}

void addPointAs3D(std::vector<float> &bezier, float x, float y)
{

    bezier.push_back(x);
    bezier.push_back(y);
    bezier.push_back(0.0f);
}

void calculateTangentVisuals(const std::vector<point2d> &B, const std::vector<point2d> &T)
{
    tangentLines.clear();
    if (!showTangents)
        return;

    for (size_t i = 0; i < B.size(); ++i)
    {

        point2d handle_in = {B[i].x - T[i].x / 3.0f, B[i].y - T[i].y / 3.0f};
        point2d handle_out = {B[i].x + T[i].x / 3.0f, B[i].y + T[i].y / 3.0f};

        addPointAs3D(tangentLines, handle_in.x, handle_in.y);
        addPointAs3D(tangentLines, handle_out.x, handle_out.y);
    }
}
void calculatePiecewiseBezier()
{
    // TODO

    // processing control points
    piecewiseBezier.clear();

    int m = (controlPoints.size());
    if (m < 2 * 3) // checking if <=2 vertices
        return;

    std::vector<point2d> B; // vector holding Bezier curve control points
    B.reserve(m / 3);       // using reserve to allocate capacity
    for (int i = 0; i + 2 < m; i += 3)
    {
        B.push_back({controlPoints[i], controlPoints[i + 1]});
    }
    int n = (B.size()) - 1; // last index
    if (n <= 0)             // checking if only one point
        return;
    std::vector<point2d> T(B.size());

    // calculating tangents points
    if (B.size() == 2) // if only 2 segments
    {
        T[0] = {B[1].x - B[0].x, B[1].y - B[0].y}; // finite difference
        T[1] = {B[1].x - B[0].x, B[1].y - B[0].y};
    }
    else
    {
        T[0] = {B[1].x - B[0].x, B[1].y - B[0].y};
        T[n] = {B[n].x - B[n - 1].x, B[n].y - B[n - 1].y};
        for (int i = 1; i < n; i++)
        {
            T[i] = {(B[i + 1].x - B[i - 1].x) * 0.5f, // using central difference
                    (B[i + 1].y - B[i - 1].y) * 0.5f};
        }
    }

    // Calculate the visuals for the tangents
    calculateTangentVisuals(B, T);
    // For each segment, build cubic Bezier and sample
    int samples = std::max(2, SAMPLES_PER_BEZIER);
    float interval = 1.0f / (samples - 1);
    for (int i = 0; i < n; ++i)
    {
        // Control points B0..B3 for the cubic between B[i] and B[i+1]
        point2d B0 = B[i];
        point2d B3 = B[i + 1];
        point2d B1 = {B[i].x + T[i].x / 3.0f, B[i].y + T[i].y / 3.0f};
        point2d B2 = {B[i + 1].x - T[i + 1].x / 3.0f, B[i + 1].y - T[i + 1].y / 3.0f};

        // Sample [0,1] and append to piecewiseBezier (using parametric equation  t from 0 to 1)
        for (int k = 0; k < samples; ++k)
        {
            float t = k * interval;
            float v = 1.0f - t;
            float b0 = v * v * v;
            float b1 = 3.0f * v * v * t;
            float b2 = 3.0f * v * t * t;
            float b3 = t * t * t;

            float x = b0 * B0.x + b1 * B1.x + b2 * B2.x + b3 * B3.x;
            float y = b0 * B0.y + b1 * B1.y + b2 * B2.y + b3 * B3.y;

            if (i > 0 && k == 0)
                continue;

            addPointAs3D(piecewiseBezier, x, y);
        }
    }
}
int main(int, char *argv[])
{
    GLFWwindow *window = setupWindow(width, height);
    ImGuiIO &io = ImGui::GetIO(); // Create IO object

    ImVec4 clear_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

    unsigned int shaderProgram = createProgram("./shaders/vshader.vs", "./shaders/fshader.fs");
    glUseProgram(shaderProgram);

    // Create VBOs, VAOs
    unsigned int VBO_controlPoints, VBO_controlPolyline, VBO_piecewiseBezier;
    unsigned int VAO_controlPoints, VAO_controlPolyline, VAO_piecewiseBezier;
    glGenBuffers(1, &VBO_controlPoints);
    glGenVertexArrays(1, &VAO_controlPoints);
    glGenBuffers(1, &VBO_controlPolyline);
    glGenVertexArrays(1, &VAO_controlPolyline);
    // TODO:
    glGenBuffers(1, &VBO_piecewiseBezier);
    glGenVertexArrays(1, &VAO_piecewiseBezier);

    glGenBuffers(1, &VBO_tangentLines);
    glGenVertexArrays(1, &VAO_tangentLines);
    int button_status = 0;

    // Display loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Options");
        ImGui::Checkbox("Show Tangents", &showTangents);
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            controlPointsUpdated = true; // Redraw when toggled
        }
        ImGui::End();
        // Rendering
        showOptionsDialog(controlPoints, io);
        ImGui::Render();

        // Add a new point on mouse click
        float x, y;
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        if (!ImGui::IsAnyItemActive())
        {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                x = io.MousePos.x;
                y = io.MousePos.y;
                if (!controlPointsFinished)
                { // Add points
                    addControlPoint(controlPoints, x, y, width, height);
                    controlPointsUpdated = true;
                }
                else
                { // Select point
                    searchNearestControlPoint(x, y);
                }
            }

            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && controlPointsFinished)
            { // Edit points
                if (selectedControlPoint >= 0)
                {
                    x = io.MousePos.x;
                    y = io.MousePos.y;
                    editControlPoint(controlPoints, x, y, width, height);
                    controlPointsUpdated = true;
                }
            }

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
            { // Stop adding points
                controlPointsFinished = true;
            }
        }

        if (controlPointsUpdated)
        {
            // Update VAO/VBO for control points
            glBindVertexArray(VAO_controlPoints);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_controlPoints);
            glBufferData(GL_ARRAY_BUFFER, controlPoints.size() * sizeof(GLfloat), &controlPoints[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0); // Enable first attribute buffer (vertices)

            // Update VAO/VBO for the control polyline
            calculateControlPolyline();
            glBindVertexArray(VAO_controlPolyline);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_controlPolyline);
            glBufferData(GL_ARRAY_BUFFER, controlPolyline.size() * sizeof(GLfloat), &controlPolyline[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0); // Enable first attribute buffer (vertices)

            // Update VAO/VBO for piecewise Bezier curve
            // TODO:
            calculatePiecewiseBezier();
            glBindVertexArray(VAO_piecewiseBezier);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_piecewiseBezier);
            glBufferData(GL_ARRAY_BUFFER,
                         piecewiseBezier.size() * sizeof(GLfloat),
                         piecewiseBezier.empty() ? nullptr : &piecewiseBezier[0],
                         GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0);

            glBindVertexArray(VAO_tangentLines);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_tangentLines);
            glBufferData(GL_ARRAY_BUFFER, tangentLines.size() * sizeof(GLfloat), tangentLines.empty() ? nullptr : &tangentLines[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0);
            controlPointsUpdated = false; // Finish all VAO/VBO updates before setting this to false.
        }

        glUseProgram(shaderProgram);

        // Draw control points
        glBindVertexArray(VAO_controlPoints);
        glDrawArrays(GL_POINTS, 0, controlPoints.size() / 3); // Draw points

#if DRAW_PIECEWISE_BEZIER
        // TODO:
        glBindVertexArray(VAO_piecewiseBezier);
        glDrawArrays(GL_LINE_STRIP, 0, piecewiseBezier.size() / 3);
#else
        // Draw control polyline
        glBindVertexArray(VAO_controlPolyline);
        glDrawArrays(GL_LINE_STRIP, 0, controlPolyline.size() / 3); // Draw lines
#endif
        if (showTangents)
        {
            glBindVertexArray(VAO_tangentLines);
            glDrawArrays(GL_LINES, 0, tangentLines.size() / 3);
        }

        // Draw control points on top
        glBindVertexArray(VAO_controlPoints);
        glDrawArrays(GL_POINTS, 0, controlPoints.size() / 3);
        glUseProgram(0);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Delete VBO buffers
    glDeleteBuffers(1, &VBO_controlPoints);
    glDeleteBuffers(1, &VBO_controlPolyline);
    glDeleteBuffers(1, &VBO_piecewiseBezier);
    glDeleteBuffers(1, &VBO_tangentLines);
    // Delete VAOs
    glDeleteVertexArrays(1, &VAO_controlPoints);
    glDeleteVertexArrays(1, &VAO_controlPolyline);
    glDeleteVertexArrays(1, &VAO_piecewiseBezier);
    glDeleteVertexArrays(1, &VAO_tangentLines);
    // Cleanup
    cleanup(window);
    return 0;
}
