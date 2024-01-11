package com.team1701.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Notifier;

public class SignalSamplingThread {
    private final Lock mSignalsLock = new ReentrantLock();
    private final Lock mPhoenixSignalsLock = new ReentrantLock();
    private final List<Runnable> mSignalQueues = new ArrayList<>();
    private final Notifier mNotifier;
    private final double mFrequency;

    private BaseStatusSignal[] mPhoenixSignals = new BaseStatusSignal[] {};

    public SignalSamplingThread(String name, double frequency) {
        mNotifier = new Notifier(this::periodic);
        mNotifier.setName(name);
        mNotifier.startPeriodic(1.0 / frequency);
        mFrequency = frequency;
    }

    public Lock getLock() {
        return mSignalsLock;
    }

    public double getFrequency() {
        return mFrequency;
    }

    public Queue<Double> addSignal(StatusSignal<Double> signal) {
        // 100 samples should be plenty for any use cases
        var queue = new ArrayBlockingQueue<Double>(100);

        mPhoenixSignalsLock.lock();
        try {
            mSignalsLock.lock();
            try {
                mSignalQueues.add(new PhoenixSignalQueue(queue, signal));
                var newSignals = new BaseStatusSignal[mPhoenixSignals.length + 1];
                System.arraycopy(mPhoenixSignals, 0, newSignals, 0, mPhoenixSignals.length);
                newSignals[mPhoenixSignals.length] = signal;
                mPhoenixSignals = newSignals;
            } finally {
                mSignalsLock.unlock();
            }
        } finally {
            mPhoenixSignalsLock.unlock();
        }

        return queue;
    }

    public Queue<Double> addSignal(DoubleSupplier signal) {
        var queue = new ArrayBlockingQueue<Double>(100);
        mSignalsLock.lock();
        try {
            mSignalQueues.add(new BasicSignalQueue(queue, signal));
        } finally {
            mSignalsLock.unlock();
        }

        return queue;
    }

    private void periodic() {
        mPhoenixSignalsLock.lock();
        try {
            if (mPhoenixSignals.length > 0) {
                BaseStatusSignal.refreshAll(mPhoenixSignals);
            }
        } finally {
            mPhoenixSignalsLock.unlock();
        }

        mSignalsLock.lock();
        try {
            mSignalQueues.forEach(Runnable::run);
        } finally {
            mSignalsLock.unlock();
        }
    }

    private class BasicSignalQueue implements Runnable {
        private final Queue<Double> queue;
        private final DoubleSupplier signal;

        public BasicSignalQueue(Queue<Double> queue, DoubleSupplier signal) {
            this.queue = queue;
            this.signal = signal;
        }

        public void run() {
            queue.offer(signal.getAsDouble());
        }
    }

    private class PhoenixSignalQueue implements Runnable {
        private final Queue<Double> queue;
        private final StatusSignal<Double> signal;

        public PhoenixSignalQueue(Queue<Double> queue, StatusSignal<Double> signal) {
            this.queue = queue;
            this.signal = signal;
        }

        public void run() {
            queue.offer(signal.getValue());
        }
    }
}
