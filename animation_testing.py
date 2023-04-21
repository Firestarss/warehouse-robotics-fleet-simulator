import numpy as np
import plotly.graph_objects as go

x = list(range(10))
y = list(range(10))
z = list(range(10))
ts = list(range(10))


# Create figure
fig = go.Figure(
    data=[go.Scatter3d(x=[x[0]], y=[y[0]], z=[z[0]],
                     mode="markers",marker=dict(color="red", size=10))])


# make figure
fig_dict = {
    "data": [],
    "layout": {},
    "frames": []
}

layout_dict = {}

# fill in most of layout
# layout_dict["xaxis"] = {"range": [30, 85], "title": "Life Expectancy"}
# layout_dict["yaxis"] = {"title": "GDP per Capita", "type": "log"}
# layout_dict["hovermode"] = "closest"
layout_dict["updatemenus"] = [
    {
        "buttons": [
            {
                "args": [None, {"frame": {"duration": 500, "redraw": True},
                                "fromcurrent": True, "transition": {"duration": 300,
                                                                    "easing": "quadratic-in-out"}}],
                "label": "Play",
                "method": "animate"
            },
            {
                "args": [[None], {"frame": {"duration": 0, "redraw": True},
                                "mode": "immediate",
                                "transition": {"duration": 0}}],
                "label": "Pause",
                "method": "animate"
            }
        ],
        "direction": "left",
        "pad": {"r": 10, "t": 87},
        "showactive": False,
        "type": "buttons",
        "x": 0.1,
        "xanchor": "right",
        "y": 0,
        "yanchor": "top"
    }
]

sliders_dict = {
    "active": 0,
    "yanchor": "top",
    "xanchor": "left",
    "currentvalue": {
        "font": {"size": 20},
        "prefix": "Year:",
        "visible": True,
        "xanchor": "right"
    },
    "transition": {"duration": 300, "easing": "cubic-in-out"},
    "pad": {"b": 10, "t": 50},
    "len": 0.9,
    "x": 0.1,
    "y": 0,
    "steps": []
}


# make frames
for t in ts:
    slider_step = {"args": [
        [t],
        {"frame": {"duration": 300, "redraw": True},
        "mode": "immediate",
        "transition": {"duration": 300}}
    ],
        "label": t,
        "method": "animate"}
    sliders_dict["steps"].append(slider_step)


layout_dict["sliders"] = [sliders_dict]

    
fig.update_layout(scene=dict(
                        xaxis=dict(range=[min(x), max(x)], autorange=False),
                        yaxis=dict(range=[min(y), max(y)], autorange=False),
                        zaxis=dict(range=[min(z), max(z)], autorange=False),
                        ))


frames = [go.Frame(data= [go.Scatter3d(
                                       x=[x[k]], 
                                       y=[y[k]],
                                       z=[z[k]])],
                   traces= [0],
                   name=f'frame{k}'      
                  ) for k in range(len(x))]
fig.update(frames=frames)

fig.update_layout(updatemenus=layout_dict["updatemenus"])
fig.update_layout(sliders=layout_dict["sliders"])




fig.show()