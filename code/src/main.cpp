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
int width = 640, height = 640;
bool controlPointsUpdated = false;
bool controlPointsFinished = false;
int selectedControlPoint = -1;

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

        // sampled thru linear interpolation?
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

void calculatePiecewiseBezier()
{
    // TODO
    piecewiseBezier.clear();

    const int nFloats = static_cast<int>(controlPoints.size());
    if (nFloats < 2 * 3)
        return;

    // -------- 1) Collect data points Pi (as 2D) --------
    struct Pt
    {
        float x, y;
    };
    std::vector<Pt> P;
    P.reserve(nFloats / 3);
    for (int i = 0; i + 2 < nFloats; i += 3)
    {
        P.push_back({controlPoints[i], controlPoints[i + 1]});
    }
    const int n = static_cast<int>(P.size()) - 1; // last index
    if (n <= 0)
        return; // only one point
    std::vector<Pt> T(P.size());
    if (P.size() == 2)
    {
        // Only one segment: simple one-sided tangents
        T[0] = {P[1].x - P[0].x, P[1].y - P[0].y};
        T[1] = {P[1].x - P[0].x, P[1].y - P[0].y};
    }
    else
    {
        // Endpoints
        T[0] = {P[1].x - P[0].x, P[1].y - P[0].y};
        T[n] = {P[n].x - P[n - 1].x, P[n].y - P[n - 1].y};
        // Interior (uniform Catmull–Rom style)
        for (int i = 1; i < n; ++i)
        {
            T[i] = {0.5f * (P[i + 1].x - P[i - 1].x),
                    0.5f * (P[i + 1].y - P[i - 1].y)};
        }
    }

    // Convenience lambda to push a 2D point as (x,y,0)
    auto push3 = [&](float x, float y)
    {
        piecewiseBezier.push_back(x);
        piecewiseBezier.push_back(y);
        piecewiseBezier.push_back(0.0f);
    };

    // -------- 3) For each segment, build cubic Bezier and sample --------
    const int samples = std::max(2, SAMPLES_PER_BEZIER);
    const float inv = 1.0f / float(samples - 1);
    for (int i = 0; i < n; ++i)
    {
        // Control points B0..B3 for the cubic between P[i] and P[i+1]
        Pt B0 = P[i];
        Pt B3 = P[i + 1];
        Pt B1 = {P[i].x + T[i].x / 3.0f, P[i].y + T[i].y / 3.0f};
        Pt B2 = {P[i + 1].x - T[i + 1].x / 3.0f, P[i + 1].y - T[i + 1].y / 3.0f};
        // Sample [0,1] and append to piecewiseBezier
        for (int k = 0; k < samples; ++k)
        {
            float t = k * inv;
            float u = 1.0f - t;
            float b0 = u * u * u;        // (1-t)^3
            float b1 = 3.0f * u * u * t; // 3(1-t)^2 t
            float b2 = 3.0f * u * t * t; // 3(1-t) t^2
            float b3 = t * t * t;        // t^3

            float x = b0 * B0.x + b1 * B1.x + b2 * B2.x + b3 * B3.x;
            float y = b0 * B0.y + b1 * B1.y + b2 * B2.y + b3 * B3.y;

            // Avoid duplicating the first sample of subsequent segments
            if (i > 0 && k == 0)
                continue;
            push3(x, y);
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

    int button_status = 0;

    // Display loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

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
            // Update VAO/VBO for control points (since we added a new point)
            glBindVertexArray(VAO_controlPoints);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_controlPoints);
            glBufferData(GL_ARRAY_BUFFER, controlPoints.size() * sizeof(GLfloat), &controlPoints[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0); // Enable first attribute buffer (vertices)

            // Update VAO/VBO for the control polyline (since we added a new point)
            calculateControlPolyline();
            glBindVertexArray(VAO_controlPolyline);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_controlPolyline);
            glBufferData(GL_ARRAY_BUFFER, controlPolyline.size() * sizeof(GLfloat), &controlPolyline[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0); // Enable first attribute buffer (vertices)

            // Update VAO/VBO for piecewise Bezier curve
            // TODO:
            calculatePiecewiseBezier(); // <- your Bernstein/de Casteljau routine fills piecewiseBezier
            glBindVertexArray(VAO_piecewiseBezier);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_piecewiseBezier);
            glBufferData(GL_ARRAY_BUFFER,
                         piecewiseBezier.size() * sizeof(GLfloat),
                         piecewiseBezier.empty() ? nullptr : &piecewiseBezier[0],
                         GL_DYNAMIC_DRAW);
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

        glUseProgram(0);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Delete VBO buffers
    glDeleteBuffers(1, &VBO_controlPoints);
    glDeleteBuffers(1, &VBO_controlPolyline);
    // TODO:
    // Delete VBO buffers
    glDeleteBuffers(1, &VBO_controlPoints);
    glDeleteBuffers(1, &VBO_controlPolyline);
    glDeleteBuffers(1, &VBO_piecewiseBezier);

    // Delete VAOs
    glDeleteVertexArrays(1, &VAO_controlPoints);
    glDeleteVertexArrays(1, &VAO_controlPolyline);
    glDeleteVertexArrays(1, &VAO_piecewiseBezier);

    // Cleanup
    cleanup(window);
    return 0;
}
