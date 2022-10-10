import matplotlib.pyplot as plt
import serial as sr
import time


def main():
    t, vol, flow = [], [], []
    dt = 0.0000001

    plt.close('all')
    fig = plt.figure(figsize=(7, 9))
    plt.ion()
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    plt.subplots_adjust(hspace=0.3)
    plt.suptitle('Real-Time Spirometric Test')
    ax1.set_title('Volume vs Time')
    ax1.grid()
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Volume [L]')
    ax1.set_xlim([0, 6])
    ax1.set_ylim([0, 7])
    ax2.set_title('Flow vs Volume')
    ax2.grid()
    ax2.set_xlabel('Volume [L]')
    ax2.set_ylabel('Flow [L/s]')
    ax2.set_xlim([0, 7])
    ax2.set_ylim([0, 4])
    plt.show()
    plt.pause(dt)

    try:
        s = sr.Serial('COM4', 115200, timeout=None, parity='N', stopbits=1, rtscts=0)
        s.reset_input_buffer()
        s.reset_output_buffer()
        start_time = 0
        i = 1
        first = True
        while True:
            txt = s.readline().decode().strip()
            print(txt)
            if first:
                first = False
                start_time = time.time()
            else:
                i = i + 1
            if txt == 'EOF':
                print('Total time registered from python: ', time.time() - start_time)
                fev1 = float(s.readline().decode().strip())
                fvc = float(s.readline().decode().strip())
                ratio = float(s.readline().decode().strip())

                ax1.axvline(x=1, linestyle='--', color='k')
                ax1.plot(1, fev1, 'ro', label=f'FEV1 = {fev1}')
                ax1.plot(max(t), fvc, 'go', label=f'FVC = {fvc}')
                txt = ax1.text(1, fev1, f'  FEV1/FVC = {ratio}%', ha='left', va='center', wrap=True)
                ax1.legend(loc='best')
                plt.savefig('spirometric-test.png', format='png')
                plt.ioff()
                plt.show()
                break
            array = txt.split(',')
            t.append(float(array[0]))
            vol.append(float(array[1]))
            flow.append(float(array[2]))

            if i % 20 == 0:
                ax1.cla()
                ax1.set_title('Volume vs Time')
                ax1.grid()
                ax1.set_xlabel('Time [s]')
                ax1.set_ylabel('Volume [L]')
                ax1.set_xlim([0, 6])
                ax1.set_ylim([0, max([7] + vol)])
                ax1.plot(t, vol)
                ax2.cla()
                ax2.set_title('Flow vs Volume')
                ax2.grid()
                ax2.set_xlabel('Volume [L]')
                ax2.set_ylabel('Flow [L/s]')
                ax2.set_xlim([0, max([7] + vol)])
                ax2.set_ylim([min([0] + flow), max([4] + flow)])
                ax2.plot(vol, flow)
                plt.pause(dt)
        s.flush()
        s.close()

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
