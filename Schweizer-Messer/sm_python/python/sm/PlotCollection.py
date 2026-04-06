# import wxversion
# wxversion.ensureMinimal('2.8')

import collections

try:
    import wx
    import wx.aui
    import matplotlib as mpl
    from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as Canvas
    from matplotlib.backends.backend_wx import NavigationToolbar2Wx as Toolbar
    _WX_AVAILABLE = True
except ImportError:
    _WX_AVAILABLE = False

if _WX_AVAILABLE:
    class PlotCollection:
        def __init__(self, window_name="", window_size=(800,600)):
            self.frame_name = window_name
            self.window_size = window_size
            self.figureList = collections.OrderedDict()

        def add_figure(self, tabname, fig):
            self.figureList[tabname] = fig

        def delete_figure(self, name):
            self.figureList.pop(name, None)

        def show(self):
            if len(list(self.figureList.keys())) == 0:
                return
            app = wx.App()
            frame = wx.Frame(None, -1, self.frame_name, size=self.window_size)
            plotter = self.PlotNotebook(frame)
            for name in list(self.figureList.keys()):
                plotter.add(name, self.figureList[name])
            frame.Show()
            app.MainLoop()

        class Plot(wx.Panel):
            def __init__(self, parent, fig, id=-1, dpi=None, **kwargs):
                wx.Panel.__init__(self, parent, id=id, **kwargs)
                fig.set_figheight(2)
                fig.set_figwidth(2)
                self.canvas = Canvas(self, -1, fig)
                self.toolbar = Toolbar(self.canvas)
                self.toolbar.Realize()
                sizer = wx.BoxSizer(wx.VERTICAL)
                sizer.Add(self.canvas, 1, wx.EXPAND)
                sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
                self.SetSizer(sizer)

        class PlotNotebook(wx.Panel):
            def __init__(self, parent, id=-1):
                wx.Panel.__init__(self, parent, id=id)
                self.nb = wx.aui.AuiNotebook(self)
                sizer = wx.BoxSizer()
                sizer.Add(self.nb, 1, wx.EXPAND)
                self.SetSizer(sizer)

            def add(self, name, fig):
                page = PlotCollection.Plot(self.nb, fig)
                self.nb.AddPage(page, name)

else:
    class PlotCollection:
        """Stub — wx is not installed. Report display is unavailable (use --dont-show-report)."""
        def __init__(self, window_name="", window_size=(800, 600)):
            self.frame_name = window_name
            self.window_size = window_size
            self.figureList = collections.OrderedDict()

        def add_figure(self, tabname, fig):
            self.figureList[tabname] = fig

        def delete_figure(self, name):
            self.figureList.pop(name, None)

        def show(self):
            print("Warning: wx is not installed — cannot display PlotCollection window. "
                  "Use --dont-show-report to suppress this.")
