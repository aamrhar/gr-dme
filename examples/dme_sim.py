#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Dme Sim
# Generated: Mon Dec 19 09:56:45 2016
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import zeromq
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import avio505_utils
import dme
import pmt
import sip
import sys
import threading
import time


class dme_sim(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Dme Sim")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Dme Sim")
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "dme_sim")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.dme_d = dme_d = 0
        self.auto_dist = auto_dist = 0
        self.vor_freq = vor_freq = 108
        self.samp_rate = samp_rate = 1e6
        self.delay = delay = int(2*dme_d*1852*1e6/3e8)+50
        self.auto_distx = auto_distx = auto_dist

        ##################################################
        # Blocks
        ##################################################
        self._dme_d_range = Range(0, 200, 0.01, 0, 200)
        self._dme_d_win = RangeWidget(self._dme_d_range, self.set_dme_d, 'dme_d', "counter_slider", float)
        self.top_layout.addWidget(self._dme_d_win)
        self._delay_tool_bar = Qt.QToolBar(self)
        
        if None:
          self._delay_formatter = None
        else:
          self._delay_formatter = lambda x: x
        
        self._delay_tool_bar.addWidget(Qt.QLabel('delay'+": "))
        self._delay_label = Qt.QLabel(str(self._delay_formatter(self.delay)))
        self._delay_tool_bar.addWidget(self._delay_label)
        self.top_layout.addWidget(self._delay_tool_bar)
          
        self.avio505_utils_test_funcs_0 = avio505_utils.test_funcs()
        self.zeromq_pull_msg_source_1 = zeromq.pull_msg_source('tcp://127.0.0.1:1234', 100)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_f(
        	1024, #size
        	samp_rate, #samp_rate
        	"", #name
        	2 #number of inputs
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)
        
        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")
        
        self.qtgui_time_sink_x_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        
        if not True:
          self.qtgui_time_sink_x_0.disable_legend()
        
        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        
        for i in xrange(2):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])
        
        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.dme_fpga_clk32_sim_0 = dme.fpga_clk32_sim(40)
        self.dme_dme_tx_0 = dme.dme_tx(samp_rate, vor_freq,1)
        self.dme_dme_time_0 = dme.dme_time()
        self.dme_dme_rx_0 = dme.dme_rx(samp_rate, vor_freq, 1)
        self.blocks_throttle_0_0 = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.blocks_null_source_0 = blocks.null_source(gr.sizeof_float*1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vff((0.5, ))
        self.blocks_message_strobe_1_0 = blocks.message_strobe(pmt.intern("TEST"), 1000)
        self.blocks_message_strobe_1 = blocks.message_strobe(pmt.intern("TEST"), 1000)
        self.blocks_message_strobe_0 = blocks.message_strobe(pmt.intern("TEST"), 1000)
        self.blocks_message_debug_0 = blocks.message_debug()
        self.blocks_delay_0_0 = blocks.delay(gr.sizeof_float*1, int(2*dme_d*1852*1e6/3e8)+50)
        self._auto_distx_tool_bar = Qt.QToolBar(self)
        
        if None:
          self._auto_distx_formatter = None
        else:
          self._auto_distx_formatter = lambda x: x
        
        self._auto_distx_tool_bar.addWidget(Qt.QLabel('auto_dist'+": "))
        self._auto_distx_label = Qt.QLabel(str(self._auto_distx_formatter(self.auto_distx)))
        self._auto_distx_tool_bar.addWidget(self._auto_distx_label)
        self.top_layout.addWidget(self._auto_distx_tool_bar)
          
        
        def _auto_dist_probe():
            while True:
                val = self.avio505_utils_test_funcs_0.delay_stim()
                try:
                    self.set_auto_dist(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (0.2))
        _auto_dist_thread = threading.Thread(target=_auto_dist_probe)
        _auto_dist_thread.daemon = True
        _auto_dist_thread.start()
            

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.avio505_utils_test_funcs_0, 'log'), (self.blocks_message_strobe_1_0, 'set_msg'))    
        self.msg_connect((self.dme_dme_rx_0, 'dist'), (self.avio505_utils_test_funcs_0, 'in'))    
        self.msg_connect((self.dme_dme_rx_0, 'to_tx'), (self.dme_dme_tx_0, 'from_rx'))    
        self.msg_connect((self.dme_dme_time_0, 'dme2'), (self.blocks_message_strobe_0, 'set_msg'))    
        self.msg_connect((self.dme_dme_time_0, 'dme1'), (self.dme_dme_rx_0, 'tx_detector'))    
        self.msg_connect((self.dme_dme_tx_0, 'cmd_out'), (self.blocks_message_debug_0, 'print'))    
        self.msg_connect((self.zeromq_pull_msg_source_1, 'out'), (self.blocks_message_debug_0, 'print'))    
        self.msg_connect((self.zeromq_pull_msg_source_1, 'out'), (self.dme_dme_rx_0, 'cmd_in'))    
        self.connect((self.blocks_delay_0_0, 0), (self.blocks_multiply_const_vxx_0, 0))    
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.dme_dme_rx_0, 0))    
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.qtgui_time_sink_x_0, 0))    
        self.connect((self.blocks_null_source_0, 0), (self.blocks_throttle_0_0, 0))    
        self.connect((self.blocks_throttle_0, 0), (self.blocks_delay_0_0, 0))    
        self.connect((self.blocks_throttle_0, 0), (self.dme_fpga_clk32_sim_0, 0))    
        self.connect((self.blocks_throttle_0_0, 0), (self.dme_fpga_clk32_sim_0, 1))    
        self.connect((self.dme_dme_rx_0, 0), (self.qtgui_time_sink_x_0, 1))    
        self.connect((self.dme_dme_time_0, 0), (self.dme_dme_rx_0, 1))    
        self.connect((self.dme_dme_tx_0, 0), (self.blocks_throttle_0, 0))    
        self.connect((self.dme_fpga_clk32_sim_0, 0), (self.dme_dme_time_0, 0))    

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "dme_sim")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_dme_d(self):
        return self.dme_d

    def set_dme_d(self, dme_d):
        self.dme_d = dme_d
        self.set_delay(self._delay_formatter(int(2*self.dme_d*1852*1e6/3e8)+50))
        self.blocks_delay_0_0.set_dly(int(2*self.dme_d*1852*1e6/3e8)+50)

    def get_auto_dist(self):
        return self.auto_dist

    def set_auto_dist(self, auto_dist):
        self.auto_dist = auto_dist
        self.set_auto_distx(self._auto_distx_formatter(self.auto_dist))

    def get_vor_freq(self):
        return self.vor_freq

    def set_vor_freq(self, vor_freq):
        self.vor_freq = vor_freq
        self.dme_dme_tx_0.Set_Mode(self.vor_freq)
        self.dme_dme_rx_0.set_mode(self.vor_freq)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.blocks_throttle_0_0.set_sample_rate(self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        Qt.QMetaObject.invokeMethod(self._delay_label, "setText", Qt.Q_ARG("QString", str(self.delay)))

    def get_auto_distx(self):
        return self.auto_distx

    def set_auto_distx(self, auto_distx):
        self.auto_distx = auto_distx
        Qt.QMetaObject.invokeMethod(self._auto_distx_label, "setText", Qt.Q_ARG("QString", eng_notation.num_to_str(self.auto_distx)))


def main(top_block_cls=dme_sim, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
