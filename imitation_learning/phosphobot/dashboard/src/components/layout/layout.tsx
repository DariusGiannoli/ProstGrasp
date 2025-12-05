import { ErrorBoundary } from "@/components/common/error";
import { Footer } from "@/components/layout/footer";
import { Sidebar } from "@/components/layout/sidebar";
import { TopBar } from "@/components/layout/topbar";
import { Toaster } from "@/components/ui/sonner";
import { Outlet } from "react-router-dom";

export function Layout() {
  return (
    <div className="flex flex-col h-screen bg-muted overflow-hidden">
      <TopBar />
      <div className="flex flex-1 overflow-hidden">
        <Sidebar />
        <main className="flex-1 container mx-auto py-6 px-4 md:px-6 overflow-y-auto pt-[150px] md:pt-[100px] pb-[60px]">
          <ErrorBoundary>
            <Outlet />
          </ErrorBoundary>
        </main>
        <Toaster position="top-center" />
      </div>
      <Footer />
    </div>
  );
}
