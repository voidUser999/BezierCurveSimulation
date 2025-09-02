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
#define SAMPLES_PER_BEZIER 100  // Sample each Bezier curve as N=10 segments and draw as connected lines

// GLobal variables
std::vector<float> controlPoints;
std::vector<float> controlPolyline;
std::vector<float> piecewiseBezier;
int width = 640, height = 640;
bool controlPointsUpdated = false;
bool controlPointsFinished = false;
int selectedControlPoint = -1;

// --- Tangent editing state ---
std::vector<float> userTangents;           // 3 floats per point: (tx, ty, 0) in NDC
std::vector<unsigned char> hasUserTangent; // 0/1 per control point
int selectedTangent = -1;                  // which control point's tangent is being dragged
bool showTangents = true;                  // toggle to show/hide handles

// Visualization geometry for tangents
std::vector<float> tangentLines;  // line segments Pi -> Hi (Hi = Pi + Ti/3)
std::vector<float> tangentPoints; // handle endpoints Hi as points

// GL objects for tangent viz
unsigned int VBO_tangentLines = 0, VAO_tangentLines = 0;
unsigned int VBO_tangentPoints = 0, VAO_tangentPoints = 0;
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
static inline void ensureTangentArraysSized()
{
    const size_t n = controlPoints.size() / 3;
    if (userTangents.size() != controlPoints.size())
        userTangents.assign(controlPoints.size(), 0.0f);
    if (hasUserTangent.size() != n)
        hasUserTangent.assign(n, 0);
}

static inline ImVec2 screenToNDC(float sx, float sy, int W, int H)
{
    // NDC: x,y in [-1,1], +y up
    float x = 2.0f * (sx / float(W)) - 1.0f;
    float y = 1.0f - 2.0f * (sy / float(H));
    return ImVec2(x, y);
}

static inline float dist2(float ax, float ay, float bx, float by)
{
    float dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

static void rebuildTangentViz()
{
    tangentLines.clear();
    tangentPoints.clear();

    const int nPts = (int)controlPoints.size() / 3;
    if (nPts == 0)
        return;

    ensureTangentArraysSized();

    // Collect positions as 2D
    std::vector<ImVec2> P(nPts);
    for (int i = 0; i < nPts; ++i)
    {
        P[i].x = controlPoints[3 * i + 0];
        P[i].y = controlPoints[3 * i + 1];
    }

    // Effective tangents T (auto, then override with user)
    std::vector<ImVec2> T(nPts, ImVec2(0, 0));
    if (nPts == 1)
    {
        T[0] = ImVec2(0, 0);
    }
    else if (nPts == 2)
    {
        ImVec2 t(P[1].x - P[0].x, P[1].y - P[0].y);
        T[0] = t;
        T[1] = t;
    }
    else
    {
        // One-sided at ends
        T[0] = ImVec2(P[1].x - P[0].x, P[1].y - P[0].y);
        T[nPts - 1] = ImVec2(P[nPts - 1].x - P[nPts - 2].x, P[nPts - 1].y - P[nPts - 2].y);
        // Central difference (with 1/2 factor) inside
        for (int i = 1; i < nPts - 1; ++i)
        {
            T[i] = ImVec2(0.5f * (P[i + 1].x - P[i - 1].x),
                          0.5f * (P[i + 1].y - P[i - 1].y));
        }
    }
    // Override with user-set tangents
    for (int i = 0; i < nPts; ++i)
    {
        if (hasUserTangent[i])
        {
            T[i].x = userTangents[3 * i + 0];
            T[i].y = userTangents[3 * i + 1];
        }
    }

    // Build viz: line Pi->Hi and a point at Hi (Hi = Pi + T/3)
    for (int i = 0; i < nPts; ++i)
    {
        ImVec2 Pi = P[i];
        ImVec2 Hi = ImVec2(Pi.x + T[i].x / 3.0f, Pi.y + T[i].y / 3.0f);

        tangentLines.push_back(Pi.x);
        tangentLines.push_back(Pi.y);
        tangentLines.push_back(0.0f);
        tangentLines.push_back(Hi.x);
        tangentLines.push_back(Hi.y);
        tangentLines.push_back(0.0f);

        tangentPoints.push_back(Hi.x);
        tangentPoints.push_back(Hi.y);
        tangentPoints.push_back(0.0f);
    }
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
        // Endpoints (one-sided)
        T[0] = {P[1].x - P[0].x, P[1].y - P[0].y};
        T[n] = {P[n].x - P[n - 1].x, P[n].y - P[n - 1].y};
        // Interior (central difference with 1/2)
        for (int i = 1; i < n; ++i)
        {
            T[i] = {0.5f * (P[i + 1].x - P[i - 1].x),
                    0.5f * (P[i + 1].y - P[i - 1].y)};
        }
    }

    // --- NEW: override with user-set tangents (if any) ---
    ensureTangentArraysSized();
    for (int i = 0; i <= n; ++i)
    {
        if (i < (int)hasUserTangent.size() && hasUserTangent[i])
        {
            T[i].x = userTangents[3 * i + 0];
            T[i].y = userTangents[3 * i + 1];
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
    glGenBuffers(1, &VBO_tangentLines);
    glGenVertexArrays(1, &VAO_tangentLines);

    glGenBuffers(1, &VBO_tangentPoints);
    glGenVertexArrays(1, &VAO_tangentPoints);

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
        // --- Tangent editing (Shift + click/drag) ---
        if (!ImGui::IsAnyItemActive())
        {
            int W = display_w, H = display_h;

            // --- Phase A: BEFORE finishing, let user add points with L-click ---
            if (!controlPointsFinished)
            {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                {
                    float sx = io.MousePos.x, sy = io.MousePos.y;
                    addControlPoint(controlPoints, sx, sy, width, height);
                    controlPointsUpdated = true;
                }

                if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                {
                    controlPointsFinished = true; // switch to edit/tangent mode
                }
            }
            else
            {
                // --- Phase B: AFTER finishing, allow point edit and tangent edit ---

                // (1) Select a control point to edit (existing app behavior)
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                {
                    searchNearestControlPoint(io.MousePos.x, io.MousePos.y);
                }

                // (2) Drag to edit control point position
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && selectedControlPoint >= 0)
                {
                    editControlPoint(controlPoints, io.MousePos.x, io.MousePos.y, width, height);
                    controlPointsUpdated = true;
                }

                // (3) SHIFT + click/drag to edit tangents (io-based)
                ensureTangentArraysSized();

                bool shiftDown = io.KeyShift;
                bool leftClicked = io.MouseClicked[ImGuiMouseButton_Left];
                bool leftDown = io.MouseDown[ImGuiMouseButton_Left];
                bool leftReleased = io.MouseReleased[ImGuiMouseButton_Left];
                bool leftDragging = leftDown && (io.MouseDelta.x * io.MouseDelta.x + io.MouseDelta.y * io.MouseDelta.y > 0.0f);

                // SHIFT + click: pick nearest control point for tangent editing
                if (shiftDown && leftClicked)
                {
                    ImVec2 ndc = screenToNDC(io.MousePos.x, io.MousePos.y, W, H);
                    const int nPts = (int)controlPoints.size() / 3;
                    float bestD2 = 1e9f;
                    int bestIdx = -1;
                    for (int i = 0; i < nPts; ++i)
                    {
                        float px = controlPoints[3 * i + 0], py = controlPoints[3 * i + 1];
                        float d2 = dist2(ndc.x, ndc.y, px, py);
                        if (d2 < bestD2)
                        {
                            bestD2 = d2;
                            bestIdx = i;
                        }
                    }
                    selectedTangent = bestIdx;
                }

                // SHIFT + drag: set Ti = mouseNDC - Pi
                if (shiftDown && leftDragging && selectedTangent >= 0)
                {
                    ImVec2 ndc = screenToNDC(io.MousePos.x, io.MousePos.y, W, H);
                    int i = selectedTangent;
                    float px = controlPoints[3 * i + 0], py = controlPoints[3 * i + 1];
                    float tx = ndc.x - px, ty = ndc.y - py;

                    userTangents[3 * i + 0] = tx;
                    userTangents[3 * i + 1] = ty;
                    userTangents[3 * i + 2] = 0.0f;
                    hasUserTangent[i] = 1;
                    controlPointsUpdated = true;
                }

                if (leftReleased)
                    selectedTangent = -1;
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
        // --- Tangent viz ---
        if (showTangents)
        {
            rebuildTangentViz();

            glBindVertexArray(VAO_tangentLines);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_tangentLines);
            glBufferData(GL_ARRAY_BUFFER,
                         tangentLines.size() * sizeof(GLfloat),
                         tangentLines.empty() ? nullptr : tangentLines.data(),
                         GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0);

            glBindVertexArray(VAO_tangentPoints);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_tangentPoints);
            glBufferData(GL_ARRAY_BUFFER,
                         tangentPoints.size() * sizeof(GLfloat),
                         tangentPoints.empty() ? nullptr : tangentPoints.data(),
                         GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
            glEnableVertexAttribArray(0);
        }
        // Draw tangent handles (optional)

        glUseProgram(shaderProgram);

        // Draw control points
        glBindVertexArray(VAO_controlPoints);
        glDrawArrays(GL_POINTS, 0, controlPoints.size() / 3); // Draw points
        if (showTangents)
        {
            // Lines Pi->Hi
            glBindVertexArray(VAO_tangentLines);
            glDrawArrays(GL_LINES, 0, tangentLines.size() / 3);

            // Handle endpoints Hi
            glBindVertexArray(VAO_tangentPoints);
            glDrawArrays(GL_POINTS, 0, tangentPoints.size() / 3);
        }

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

    glDeleteBuffers(1, &VBO_piecewiseBezier);

    // Delete VAOs
    glDeleteVertexArrays(1, &VAO_controlPoints);
    glDeleteVertexArrays(1, &VAO_controlPolyline);
    glDeleteVertexArrays(1, &VAO_piecewiseBezier);

    // Cleanup
    glDeleteBuffers(1, &VBO_tangentLines);
    glDeleteVertexArrays(1, &VAO_tangentLines);
    glDeleteBuffers(1, &VBO_tangentPoints);
    glDeleteVertexArrays(1, &VAO_tangentPoints);

    cleanup(window);
    return 0;
}
