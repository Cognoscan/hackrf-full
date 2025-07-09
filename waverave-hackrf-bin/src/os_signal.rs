use tokio_util::sync::CancellationToken;

static CANCEL: tokio::sync::OnceCell<CancellationToken> = tokio::sync::OnceCell::const_new();

pub struct Quit(CancellationToken);

impl Quit {
    /// Returns when the program has been asked to quit by the OS.
    pub async fn quit(&self) {
        self.0.cancelled().await
    }

    pub fn cancel(&self) {
        self.0.cancel();
    }

    pub async fn run_until_cancelled<F>(&self, fut: F) -> Option<F::Output>
    where
        F: Future,
    {
        self.0.run_until_cancelled(fut).await
    }

    pub async fn run_until_cancelled_owned<F>(self, fut: F) -> Option<F::Output>
    where
        F: Future,
    {
        self.0.run_until_cancelled_owned(fut).await
    }
}

pub async fn quit_watch() -> Quit {
    let cancel = CANCEL
        .get_or_init(|| async move {
            let cancel = CancellationToken::new();
            #[cfg(unix)]
            {
                use tokio::signal::unix;

                // SIGINT
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    unix::signal(unix::SignalKind::interrupt())
                        .unwrap()
                        .recv()
                        .await;
                    cancel_tx.cancel();
                });

                // SIGHUP
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    unix::signal(unix::SignalKind::hangup())
                        .unwrap()
                        .recv()
                        .await;
                    cancel_tx.cancel();
                });

                // SIGTERM
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    unix::signal(unix::SignalKind::terminate())
                        .unwrap()
                        .recv()
                        .await;
                    cancel_tx.cancel();
                });

                // SIGPIPE
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    unix::signal(unix::SignalKind::pipe()).unwrap().recv().await;
                    cancel_tx.cancel();
                });

                // SIGQUIT
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    unix::signal(unix::SignalKind::quit()).unwrap().recv().await;
                    cancel_tx.cancel();
                });
            }
            #[cfg(windows)]
            {
                use tokio::signal::windows;

                // ctrl-break
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    windows::ctrl_break().unwrap().recv().await;
                    cancel_tx.cancel();
                });

                // ctrl-c
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    windows::ctrl_c().unwrap().recv().await;
                    cancel_tx.cancel();
                });

                // ctrl-close
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    windows::ctrl_close().unwrap().recv().await;
                    cancel_tx.cancel();
                });

                // ctrl-logoff
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    windows::ctrl_logoff().unwrap().recv().await;
                    cancel_tx.cancel();
                });

                // ctrl-shutdown
                let cancel_tx = cancel.clone();
                tokio::spawn(async move {
                    windows::ctrl_shutdown().unwrap().recv().await;
                    cancel_tx.cancel();
                });
            }
            cancel
        })
        .await;
    let cancel = cancel.clone();

    Quit(cancel)
}
