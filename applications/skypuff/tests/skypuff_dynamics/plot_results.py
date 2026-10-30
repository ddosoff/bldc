"""Publication-style figures for the explicitly parameterized mechanical model."""
import os
import numpy as np


def plot_results(output, rows, variants, shapes, report):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p, derived = report['parameters'], report['derived']
    cfg, result = report['controller_settings'], report['metrics']
    blue, orange, green, purple = '#2563b8', '#d57724', '#208461', '#8053aa'
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':11,
        'axes.spines.top':False, 'axes.spines.right':False,
        'axes.titleweight':'bold', 'axes.titlecolor':'#1d2d40',
        'axes.edgecolor':'#b7c3cf', 'grid.color':'#e0e6ed', 'grid.linewidth':.7,
        'xtick.color':'#4d5b6c', 'ytick.color':'#4d5b6c', 'svg.fonttype':'none'})

    def data(records, key):
        return np.array([r[key] for r in records])

    def save(fig, name):
        for ext in ['png','svg']:
            fig.savefig(output/f'{name}.{ext}', dpi=170, facecolor='white')
        plt.close(fig)
        print(f'Plot: {output/(name+".png")}', flush=True)

    t = data(rows,'time_s')
    end = t[-1]
    fig, axes = plt.subplots(1,3,figsize=(18.5,7.0))
    fig.subplots_adjust(left=.055,right=.975,bottom=.24,top=.72,wspace=.28)
    fig.suptitle('Pull → Manual_braking: барабан и упругий трос', fontsize=22,fontweight='bold',y=.96,color='#162a40')
    fig.text(.5,.885,
        f'{cfg["pull_kg"]:g} → {cfg["brake_kg"]:g} кгс напрямую  ·  p = {cfg["smooth_motor_p"]:g}  ·  '
        f'{cfg["brake_ms"]/1000:g} с  ·  параплан: 7 м/с вдоль троса',
        ha='center',fontsize=13,color='#3e5269')
    fig.text(.5,.825,
        'Барабан: 15 кг на Ø250 мм, J = 0,234 кг·м²  ·  Dyneema Ø1,7 мм: 1500 м, 15°  ·  жёсткость по каталогу Armare',
        ha='center',fontsize=11,color='#536376')
    command = data(rows,'command_kgf')
    modes = data(rows,'braking').astype(bool)
    axes[0].step(t,np.where(~modes,command,np.nan),where='post',color=blue,lw=2,label='Уставка тяги')
    axes[0].step(t,np.where(modes,command,np.nan),where='post',color=orange,lw=3,label='Уставка тормоза, модуль')
    axes[0].plot(t,data(rows,'rope_force_kgf'),color=green,lw=2,label='Натяжение у барабана')
    axes[0].set_title('Сила',loc='left',pad=14)
    axes[0].set_ylabel('кгс')
    axes[0].set_ylim(0,155)
    axes[0].legend(fontsize=9,frameon=False,loc='upper right')
    axes[1].plot(t,data(rows,'speed_mps'),color=blue,lw=2,label='Поверхность барабана')
    axes[1].axhline(p['glider_speed_mps'],ls='--',color='#8793a2',lw=1.5,label='Скорость параплана')
    axes[1].axhline(0,color='#8090a2',lw=.8)
    axes[1].fill_between(t,data(rows,'speed_mps'),0,where=data(rows,'speed_mps')<0,color=blue,alpha=.10)
    axes[1].set_title('Скорость',loc='left',pad=14)
    axes[1].set_ylabel('м/с; «+» — намотка, «−» — отдача')
    axes[1].legend(fontsize=9,frameon=False,loc='best')
    axes[2].plot(t,data(rows,'acceleration_mps2'),color='#c34b45',lw=1.3,label='Механическое ускорение')
    axes[2].plot(t,data(rows,'filtered_acceleration_mps2'),color=purple,lw=2,label='После фильтров antisex')
    axes[2].axhline(0,color='#8090a2',lw=.8)
    axes[2].set_title('Ускорение',loc='left',pad=14)
    axes[2].set_ylabel('м/с² на поверхности барабана')
    axes[2].legend(fontsize=9,frameon=False,loc='best')
    for ax in axes:
        ax.set_xlabel('Время после команды, с')
        ax.set_xlim(0,max(4.15,end+.05))
        ax.grid(True)
        for event,color in [(cfg['brake_ms']/1000,orange)]:
            ax.axvline(event,ls=':',color=color,lw=1)
    fig.text(.055,.135,
        'Модель: распределённая масса троса, растяжение, провисание и сила тяжести. Уставка взята из выполнения C-кода smooth_motor.',
        fontsize=10,color='#536376')
    fig.text(.055,.09,
        'Кривая растяжения — упругая аппроксимация каталожных точек; BA = 200 Н·с задано. Antisex не управляет мотором; ветер и сопротивление воздуха не включены.',
        fontsize=10,color='#536376')
    stop = (f'Расчёт остановлен при первом касании земли на уровне лебёдки: t ≈ {end:.2f} с. Контакт с грунтом не моделируется.'
            if result['stop_reason']=='first_ground_contact' else 'Показан заданный интервал моделирования.')
    fig.text(.055,.045,stop,fontsize=10,color='#536376')
    save(fig,'pull_manual_braking_dynamics')

    fig, axes = plt.subplots(1,2,figsize=(15,6.8))
    fig.subplots_adjust(left=.08,right=.97,bottom=.23,top=.77,wspace=.28)
    fig.suptitle('Трос: равновесие под нагрузкой и каталожное растяжение',fontsize=19,fontweight='bold',y=.95)
    fig.text(.5,.875,'Dyneema SK75, аналог Armare Dynext PU Ø1,7 мм  ·  1,70 г/м  ·  начальная длина по дуге 1500 м',ha='center',fontsize=11)
    palette=['#8b6ba0','#d28a36','#2563b8','#208461']
    for state,color in zip(report['equilibria'],palette):
        r=np.array(state['coordinates'])
        deviation=r[:,1]-r[:,0]*np.tan(np.deg2rad(p['angle_deg']))
        axes[0].plot(r[:,0]/1000,deviation,color=color,lw=2,
                     label=f'{state["name"]}: {state["kgf"]:g} кгс; провис {state["sag_vertical_m"]:.1f} м')
    axes[0].set_title('Провисание относительно хорды под 15°',loc='left',fontsize=12)
    axes[0].set_xlabel('Горизонтальное расстояние, км')
    axes[0].set_ylabel('Отклонение вниз от хорды, м')
    axes[0].set_ylim(-1.4 * max(s['sag_vertical_m'] for s in report['equilibria']), .5)
    axes[0].legend(fontsize=9,frameon=False,loc='lower center')
    eps=np.array([0,p['strain_at_30_percent'],p['strain_at_break']])*100
    force=np.array([0,.3*p['breaking_load_n'],p['breaking_load_n']])/9.80665
    axes[1].plot(eps,force,color=blue,lw=2,label='Кусочно-линейная модель')
    axes[1].scatter(eps[1:],force[1:],color=orange,zorder=4,s=50,label='Две точки из каталога')
    for state,color in zip(report['equilibria'],palette):
        f=state['kgf']*9.80665
        if f<=.3*p['breaking_load_n']:
            strain=f/derived['low_ea_n']
        else:
            strain=p['strain_at_30_percent']+(f-.3*p['breaking_load_n'])/derived['high_ea_n']
        axes[1].scatter([strain*100],[state['kgf']],color=color,zorder=4)
        axes[1].annotate(state['name'],(strain*100,state['kgf']),xytext=(9,-4),textcoords='offset points',fontsize=9,color=color)
    axes[1].set_title('Сила — относительное удлинение',loc='left',fontsize=12)
    axes[1].set_xlabel('Удлинение, %')
    axes[1].set_ylabel('Натяжение, кгс')
    axes[1].legend(fontsize=9,frameon=False,loc='upper left')
    for ax in axes: ax.grid(True)
    x,z=derived['initial_endpoint_m']
    fig.text(.08,.125,f'При Pull: горизонталь {x:.1f} м, высота {z:.1f} м; растяжение материала {1500-derived["initial_rest_length_m"]:.1f} м.',fontsize=10,color='#536376')
    fig.text(.08,.075,'Источник: Armare, каталог Diving 2021, стр. 8 — 1,95% при 30% разрывной нагрузки; 3,15% при 287 даН.',fontsize=10,color='#536376')
    fig.text(.08,.03,'Интерполяция между точками и обратимость растяжения — допущения модели; реальные циклы нагружения и разгрузки требуют измерения.',fontsize=10,color='#536376')
    save(fig,'rope_equilibrium_and_stiffness')

    fig,axes=plt.subplots(1,2,figsize=(15,6.8))
    fig.subplots_adjust(left=.075,right=.975,bottom=.22,top=.78,wspace=.24)
    fig.suptitle('Влияние неизвестных параметров: затухание и инерция',fontsize=19,fontweight='bold',y=.95)
    fig.text(.5,.875,'Во всех вариантах одна каталожная кривая Dyneema; меняется только BA или распределение массы барабана',ha='center',fontsize=11)
    all_cases=[('base','Основной: BA = 200 Н·с, масса на ободе',rows)]+variants
    for (name,label,rr),color in zip(all_cases,[blue,green,purple,orange]):
        tt=data(rr,'time_s')
        for ax,key in zip(axes,['speed_mps','filtered_acceleration_mps2']):
            ax.plot(tt,data(rr,key),color=color,label=label,lw=2 if name=='base' else 1.4)
            if rr[-1]['min_height_m']<=0:
                ax.scatter([tt[-1]],[rr[-1][key]],marker='x',color=color,s=35)
    for ax,title,ylabel in zip(axes,['Скорость барабана','Ускорение после фильтров antisex'],['м/с','м/с²']):
        ax.set_title(title,loc='left',fontsize=13)
        ax.set_xlabel('Время после команды, с');ax.set_ylabel(ylabel)
        ax.grid(True);ax.axhline(0,lw=.7,color='#8b98a5')
        ax.axvline(cfg['brake_ms']/1000,ls=':',color=orange)
        ax.legend(fontsize=9,frameon=False)
    fig.text(.075,.12,'Это сравнение допущений, а не доверительный интервал. Затухание и момент инерции реальной установки не измерены.',fontsize=10,color='#536376')
    fig.text(.075,.065,'Крестик — первое касание земли: расчёт этой кривой остановлен, дальнейшее движение троса по земле не рассчитывается.',fontsize=10,color='#536376')
    save(fig,'damping_and_inertia_sensitivity')
